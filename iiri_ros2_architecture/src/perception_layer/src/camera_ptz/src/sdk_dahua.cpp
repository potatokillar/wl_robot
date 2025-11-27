#include "sdk_dahua.hpp"

#include <arpa/inet.h>
#include <ifaddrs.h>
#include <unistd.h>

#include <fstream>
#include <rclcpp/rclcpp.hpp>

#include "robot_base/timer_tools.hpp"

using namespace std;

// 当前云台运行方向
enum class Direction
{
    no,
    up,
    down,
    left,
    right
};
// 大华摄像头云台控制需要的数据
struct DahuaPtzData
{
    Direction now_direct;
    Direction next_direct;
    int x, y;
    uint64_t req_time;                        // 新方向命令的请求时间戳
    constexpr static uint64_t TIMEOUT = 300;  // 超时时间
};

class SdkDahuaImpl
{
public:
    std::set<std::string> ip_list_;
    std::function<void(const std::vector<uint8_t> &)> video_cb_, audio_cb_;
    std::string ip_;
    std::string user_;
    std::string pwd_;

public:
    SdkDahuaImpl()
    {
        CLIENT_Init(NULL, (LDWORD)this);
        LOG_SET_PRINT_INFO stLogPrintInfo = {sizeof(stLogPrintInfo)};
        CLIENT_LogOpen(&stLogPrintInfo);
    }

    virtual ~SdkDahuaImpl()
    {
        if (thread_.joinable()) {
            thread_.join();
        }
        CLIENT_Cleanup();
        if (login_handle_) {
            CLIENT_Logout(login_handle_);
        }
    }

    /**
     * @description: 搜索摄像头的回调函数
     * @return {}
     */
    static void CALLBACK search_device_callback(LLONG lSearchHandle, DEVICE_NET_INFO_EX2 *pDevNetInfo, void *pUserData)
    {
        SdkDahuaImpl *ptr = (SdkDahuaImpl *)pUserData;
        ptr->search_callback(pDevNetInfo);
    }
    void search_callback(DEVICE_NET_INFO_EX2 *pDevNetInfo)
    {
        if (pDevNetInfo != NULL) {
            std::string dev_ip(pDevNetInfo->stuDevInfo.szIP);
            std::string host_ip(pDevNetInfo->szLocalIP);

            // arm平台会返回不正常的IP
            struct in_addr addr;
            if (inet_pton(AF_INET, dev_ip.c_str(), &addr) == 1) {
                ip_list_.insert(dev_ip);
                //  当摄像头同时连入和主机一样的有线无线时，可能搜到两个IP，分别是无线网卡和有线网卡的。此处打印本机IP，由此可判断是哪个
                RCLCPP_INFO(rclcpp::get_logger("sdk_dahua"), "find it! host_ip: %s; camera_ip: %s", host_ip.c_str(), dev_ip.c_str());
            }
        }
    }

    /**
     * @description: 实时播放回调函数
     * @return {}
     */
    static void CALLBACK real_data_callback(LLONG lRealHandle, DWORD dwDataType, BYTE *pBuffer, DWORD dwBufSize, LONG param, LDWORD dwUser)
    {
        // 若多个实时预览使用相同的数据回调函数，则用户可通过 lRealHandle 进行一一对应
        SdkDahuaImpl *ptr = (SdkDahuaImpl *)dwUser;
        ptr->realplay_callback(dwDataType, pBuffer, dwBufSize, param);
    }
    void realplay_callback(DWORD dwDataType, BYTE *pBuffer, DWORD dwBufSize, LONG param)
    {
        switch (dwDataType) {
            case EM_REAL_DATA_TYPE_PRIVATE:
                // 原始音视频混合数据
                break;
            case EM_REAL_DATA_TYPE_H264 + NET_DATA_CALL_BACK_VALUE:  // 标准视频数据
            {
                std::vector<uint8_t> data(pBuffer, pBuffer + dwBufSize);
                if (video_cb_) {
                    video_cb_(data);
                }

            } break;
            case EM_REAL_DATA_TYPE_FLV_STREAM + NET_DATA_CALL_BACK_VALUE:
                // flv 数据
                break;
            case EM_REAL_DATA_TYPE_RTP + NET_DATA_CALL_BACK_VALUE:
                // rtp 音频数据
                break;
            case 4:
                // 原始音频数据
                break;
            default:
                break;
        }
    }

    /**
     * @description: 设置云台速度
     * @param msg_vx
     * @param msg_vy
     * @return {}
     */
    void set_ptz(double msg_vx, double msg_vy)
    {
        double abs_vx = abs(msg_vx);  // 速度的绝对值
        double abs_vy = abs(msg_vy);
        int value_x_vel = ValueMapping((int)(abs_vx * 100), 1, 100, 1, 8);  // 速度的等效值，是1到8的整型值
        int value_y_vel = ValueMapping((int)(abs_vy * 100), 1, 100, 1, 8);

        // 把速度等效值存放到共享区
        std::lock_guard<std::mutex> lk(ptz_mtx_);
        ptz_data_.x = value_x_vel;
        ptz_data_.y = value_y_vel;

        // 用户给的值，有可能是vx vy都有值。但本摄像头不支持同时控制
        // 这里只取大值的方向
        if ((abs_vx == 0) && (abs_vy == 0)) {
            ptz_data_.next_direct = Direction::no;
            ptz_data_.req_time = TimerTools::GetNowTickMs();
        } else if (abs_vy > abs_vx) {
            if (msg_vy > 0) {
                ptz_data_.next_direct = Direction::up;
                ptz_data_.req_time = TimerTools::GetNowTickMs();
            } else if (msg_vy < 0) {
                ptz_data_.next_direct = Direction::down;
                ptz_data_.req_time = TimerTools::GetNowTickMs();
            }
        } else if (abs_vy < abs_vx) {
            if (msg_vx > 0) {
                ptz_data_.next_direct = Direction::left;
                ptz_data_.req_time = TimerTools::GetNowTickMs();
            } else if (msg_vx < 0) {
                ptz_data_.next_direct = Direction::right;
                ptz_data_.req_time = TimerTools::GetNowTickMs();
            }
        }
    }

    /**
     * @description: 开启搜索
     * @return {}
     */
    void start_search()
    {
        ip_list_.clear();
        NET_IN_STARTSERACH_DEVICE pInBuf = {0};
        NET_OUT_STARTSERACH_DEVICE pOutBuf = {0};

        auto ipset = get_host_ip();
        for (auto it = ipset.cbegin(); it != ipset.cend(); it++) {
            pInBuf.dwSize = sizeof(NET_IN_STARTSERACH_DEVICE);
            pInBuf.cbSearchDevices = &SdkDahuaImpl::search_device_callback;
            pInBuf.pUserData = this;
            std::string ip = *it;
            strncpy(pInBuf.szLocalIp, ip.c_str(), sizeof(pInBuf.szLocalIp) - 1);
            pOutBuf.dwSize = sizeof(NET_OUT_STARTSERACH_DEVICE);
            LLONG seachHandle = CLIENT_StartSearchDevicesEx(&pInBuf, &pOutBuf);
            search_handles_.push_back(seachHandle);
        }
    }

    void stop_search()
    {
        for (int i = 0; i < search_handles_.size(); i++) {
            if (search_handles_[i] != 0) {
                CLIENT_StopSearchDevices(search_handles_[i]);  // 似乎不允许在回调中调用停止搜索
            }
        }
    }

    /**
     * @description: 登录
     * @return {} 登录成功返回true
     */
    bool login()
    {
        if (ip_.empty()) {
            return false;
        }
        RCLCPP_INFO(rclcpp::get_logger("sdk_dahua"), "camera prepare login...");

        uint32_t local_port = 37777;

        NET_IN_LOGIN_WITH_HIGHLEVEL_SECURITY stInparam;
        memset(&stInparam, 0, sizeof(stInparam));
        stInparam.dwSize = sizeof(stInparam);
        strncpy(stInparam.szIP, ip_.c_str(), sizeof(stInparam.szIP) - 1);
        strncpy(stInparam.szUserName, user_.c_str(), sizeof(stInparam.szUserName) - 1);
        strncpy(stInparam.szPassword, pwd_.c_str(), sizeof(stInparam.szPassword) - 1);

        stInparam.nPort = local_port;
        stInparam.emSpecCap = EM_LOGIN_SPEC_CAP_TCP;

        NET_OUT_LOGIN_WITH_HIGHLEVEL_SECURITY stOutparam;
        memset(&stOutparam, 0, sizeof(stOutparam));
        stOutparam.dwSize = sizeof(stOutparam);

        // 非0值，即代表登录成功
        login_handle_ = 0;
        login_handle_ = CLIENT_LoginWithHighLevelSecurity(&stInparam, &stOutparam);
        if (login_handle_ != 0) {
            RCLCPP_INFO(rclcpp::get_logger("sdk_dahua"), "camera login success, ip: %s", stInparam.szIP);
            open_realplay(login_handle_);
            start_talk();
            running_ = true;
            thread_ = std::thread([this]() { this->loop(); });
            return true;
        } else {
            RCLCPP_INFO(rclcpp::get_logger("sdk_dahua"), "camera login failed, ip: %s", stInparam.szIP);
            return false;
        }
    }

    // 语音对讲的音频数据回调函数原形(pDataBuf内存由SDK内部申请释放)
    static void CALLBACK AudioDataCallBack(LLONG lTalkHandle, char *pDataBuf, DWORD dwBufSize, BYTE byAudioFlag, LDWORD dwUser)
    {
        if (0 == byAudioFlag) {
            // 不做处理，因为是服务器模式
        } else if (1 == byAudioFlag) {
            SdkDahuaImpl *ptr = (SdkDahuaImpl *)dwUser;
            ptr->audio_callback(pDataBuf, dwBufSize);
            //  将设备发回的音频数据写到文件
        }
    }

    void audio_callback(char *pDataBuf, DWORD dwBufSize)
    {
        if (audio_cb_) {
            std::vector<uint8_t> data(pDataBuf, pDataBuf + dwBufSize);

            if (audio_cb_) {
                audio_cb_(data);
            }
        }
    }

    void send_audio(const std::vector<uint8_t> &data)
    {
        if (talk_handle_ == 0) {
            return;
        }
        NET_IN_TALK_SEND_DATA_STREAM in;
        NET_OUT_TALK_SEND_DATA_STREAM out;
        memset(&in, 0, sizeof(in));
        memset(&out, 0, sizeof(out));
        in.dwSize = sizeof(in);
        in.pBuf = (unsigned char *)data.data();
        in.dwBufSize = data.size();
        in.bNeedHead = TRUE;
        in.emEncodeType = DH_TALK_PCM;
        in.nAudioBit = 16;
        in.dwSampleRate = 8000;
        out.dwSize = sizeof(out);
        if (CLIENT_TalkSendDataByStream(talk_handle_, &in, &out) < 0) {
            RCLCPP_ERROR(rclcpp::get_logger("sdk_dahua"), "send audio error");
        }
    }

    bool start_talk()
    {
        // 未登录或者已经开启对讲，则不做处理
        if ((login_handle_ == 0) || (talk_handle_ != 0)) {
            return false;
        }

        // 设置编码方式
        DHDEV_TALKDECODE_INFO stTalkMode;
        memset(&stTalkMode, 0, sizeof(DHDEV_TALKDECODE_INFO));
        stTalkMode.encodeType = DH_TALK_PCM;  // 这里修改似乎无效，得上网页修改
        stTalkMode.dwSampleRate = 8000;       // 采样率
        stTalkMode.nAudioBit = 16;            // 位数
        BOOL bSuccess = CLIENT_SetDeviceMode(login_handle_, DH_TALK_ENCODE_TYPE, &stTalkMode);
        if (!bSuccess) {
            RCLCPP_ERROR(rclcpp::get_logger("sdk_dahua"), "set talk mode failed");
            return false;
        }

        // 设置服务器模式
        bSuccess = CLIENT_SetDeviceMode(login_handle_, DH_TALK_SERVER_MODE, NULL);
        if (!bSuccess) {
            RCLCPP_ERROR(rclcpp::get_logger("sdk_dahua"), "set talk server mode failed");
            return false;
        }

        // 设置语音参数
        NET_SPEAK_PARAM stSpackparam;
        memset(&stSpackparam, 0, sizeof(NET_SPEAK_PARAM));
        stSpackparam.dwSize = sizeof(NET_SPEAK_PARAM);
        stSpackparam.nMode = 0;
        stSpackparam.bEnableWait = false;
        stSpackparam.nSpeakerChannel = 0;
        bSuccess = CLIENT_SetDeviceMode(login_handle_, DH_TALK_SPEAK_PARAM, &stSpackparam);
        if (!bSuccess) {
            RCLCPP_ERROR(rclcpp::get_logger("sdk_dahua"), "set talk param failed");
            return false;
        }

        // 设置对象时转发模式还是直连方式
        NET_TALK_TRANSFER_PARAM stTransfer;
        memset(&stTransfer, 0, sizeof(NET_TALK_TRANSFER_PARAM));
        stTransfer.dwSize = sizeof(NET_TALK_TRANSFER_PARAM);
        stTransfer.bTransfer = 0;
        bSuccess = CLIENT_SetDeviceMode(login_handle_, DH_TALK_TRANSFER_MODE, &stTransfer);
        if (!bSuccess) {
            RCLCPP_ERROR(rclcpp::get_logger("sdk_dahua"), "set talk transfer mode failed");
            return false;
        }

        // 设置对讲通道
        int channel = 0;
        bSuccess = CLIENT_SetDeviceMode(login_handle_, DH_TALK_TALK_CHANNEL, &channel);
        if (!bSuccess) {
            RCLCPP_ERROR(rclcpp::get_logger("sdk_dahua"), "set talk channel failed");
            return false;
        }

        // 开始对讲
        talk_handle_ = CLIENT_StartTalkEx(login_handle_, AudioDataCallBack, (LDWORD)this);
        if (talk_handle_ != 0)  // 开启对讲成功
        {
            RCLCPP_INFO(rclcpp::get_logger("sdk_dahua"), "start talk");
        } else  // 开启对讲失败
        {
            RCLCPP_ERROR(rclcpp::get_logger("sdk_dahua"), "talk init failed");
            return false;
        }

        if (CLIENT_SetAudioClientVolume(talk_handle_, 100) == FALSE) {
            RCLCPP_ERROR(rclcpp::get_logger("sdk_dahua"), "talk volume failed");
        };
        return true;
    }

    bool stop_talk()
    {
        if (talk_handle_ == 0) {
            return false;
        }

        RCLCPP_INFO(rclcpp::get_logger("sdk_dahua"), "stop talk");
        CLIENT_StopTalkEx(talk_handle_);
        talk_handle_ = 0;
        return true;
    }

private:
    LLONG login_handle_ = 0;
    LLONG talk_handle_ = 0;
    std::vector<LLONG> search_handles_;  // 每一个网卡一个登录句柄
    std::mutex ptz_mtx_;
    DahuaPtzData ptz_data_;
    std::thread thread_;
    bool running_ = false;
    int channel_ = 0;
    char pCbData[102400];

private:
    /**
     * @description: 循环，目前只处理云台
     * @return {}
     */
    void loop()
    {
        DahuaPtzData ptz;
        while (running_) {
            {
                // 复制 共享区数据到局部变量
                std::lock_guard<std::mutex> lk(ptz_mtx_);
                ptz = ptz_data_;
            }

            // 当检测到req的方向改变时
            if (ptz.now_direct != ptz.next_direct) {
                // 先停止旧的运行方向
                dahua_ptz_stop(ptz.now_direct);

                // 等待停止完成？实测似乎不需要
                // std::this_thread::sleep_for(std::chrono::milliseconds(100));

                // 再运行新方向
                ptz.now_direct = ptz.next_direct;
                dahua_ptz_start(ptz.now_direct, ptz.x, ptz.y);
            } else {
                // 极端情况下，有可能未收到停止的指令，则判断超时
                if ((ptz.now_direct != Direction::no) && (TimerTools::GetNowTickMs() - ptz.req_time > ptz.TIMEOUT)) {
                    dahua_ptz_stop(ptz.now_direct);
                    ptz.next_direct = Direction::no;
                    ptz.now_direct = Direction::no;
                }
            }

            {
                // 复制局部变量到共享区
                std::lock_guard<std::mutex> lk(ptz_mtx_);
                ptz_data_ = ptz;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    };

    /**
     * @description: 打开实时预览回调
     * @param loginHandle
     * @return {}
     */
    void open_realplay(LLONG loginHandle)
    {
        NET_IN_REALPLAY_BY_DATA_TYPE stIn = {sizeof(stIn)};
        NET_OUT_REALPLAY_BY_DATA_TYPE stOut = {sizeof(stOut)};
        stIn.emDataType = EM_REAL_DATA_TYPE_H264;
        stIn.rType = DH_RType_Realplay;
        stIn.nChannelID = 0;
        stIn.hWnd = NULL;
        stIn.dwUser = (LDWORD)this;
        stIn.cbRealData = &SdkDahuaImpl::real_data_callback;
        CLIENT_RealPlayByDataType(loginHandle, &stIn, &stOut, 5000);
    }
    // 数值映射的函数，给定输入范围和输出范围，将一个输入值映射到输出范围中对应位置的值
    int ValueMapping(int input, int inputMin, int inputMax, int outputMin, int outputMax)
    {
        // 手动限制输入值在 [inputMin, inputMax] 范围内
        if (input < inputMin) {
            input = inputMin;
        } else if (input > inputMax) {
            input = inputMax;
        }

        // 计算输入范围和输出范围的长度
        int inputRange = inputMax - inputMin;
        int outputRange = outputMax - outputMin;

        // 映射
        double normalized = static_cast<double>(input - inputMin) / inputRange;
        int output = static_cast<int>(std::floor(normalized * outputRange)) + outputMin;

        return output;
    };
    /**
     * @description: 大华云台开启
     * @param direction
     * @param x
     * @param y
     * @return {}
     */
    void dahua_ptz_start(Direction direction, int x, int y)
    {
        if (login_handle_ == 0) {
            return;
        }
        switch (direction) {
            case Direction::up:
                CLIENT_DHPTZControlEx(login_handle_, channel_, DH_PTZ_UP_CONTROL, 0, y, 0, false);
                break;
            case Direction::down:
                CLIENT_DHPTZControlEx(login_handle_, channel_, DH_PTZ_DOWN_CONTROL, 0, y, 0, false);
                break;
            case Direction::left:
                CLIENT_DHPTZControlEx(login_handle_, channel_, DH_PTZ_LEFT_CONTROL, 0, x, 0, false);
                break;
            case Direction::right:
                CLIENT_DHPTZControlEx(login_handle_, channel_, DH_PTZ_RIGHT_CONTROL, 0, x, 0, false);
                break;

            default:
                break;
        }
    }

    /**
     * @description: 云台关闭
     * @param direction
     * @return {}
     */
    void dahua_ptz_stop(Direction direction)
    {
        if (login_handle_ == 0) {
            return;
        }
        switch (direction) {
            case Direction::up:
                CLIENT_DHPTZControlEx(login_handle_, channel_, DH_PTZ_UP_CONTROL, 0, 0, 0, true);
                break;
            case Direction::down:
                CLIENT_DHPTZControlEx(login_handle_, channel_, DH_PTZ_DOWN_CONTROL, 0, 0, 0, true);
                break;
            case Direction::left:
                CLIENT_DHPTZControlEx(login_handle_, channel_, DH_PTZ_LEFT_CONTROL, 0, 0, 0, true);
                break;
            case Direction::right:
                CLIENT_DHPTZControlEx(login_handle_, channel_, DH_PTZ_RIGHT_CONTROL, 0, 0, 0, true);
                break;
            default:
                break;
        }
    }

    /**
     * @description: 获取主机IP，每一个网卡都有一个IP
     * @return {}
     */
    set<std::string> get_host_ip()
    {
        struct ifaddrs *ifList = nullptr;  // getifaddrs()创建的链表上的数据结构
                                           // 获取本地网络接口的信息。
        if (getifaddrs(&ifList) < 0) {
            return {};
        }

        std::set<std::string> ipset;
        for (struct ifaddrs *ifa = ifList; ifa != nullptr; ifa = ifa->ifa_next) {
            // arm平台还需要判断ifa_addr非空
            if ((ifa->ifa_addr) && (ifa->ifa_addr->sa_family == AF_INET)) {
                if ((ifa->ifa_name == "lo") || (ifa->ifa_name == "docker0")) {
                    // 部分网口不需要，如lo和docker0
                    // continue;
                }
                struct sockaddr_in *sin = (struct sockaddr_in *)(ifa->ifa_addr);
                char ipstr[INET_ADDRSTRLEN];
                inet_ntop(AF_INET, &(sin->sin_addr), ipstr, sizeof(ipstr));
                ipset.insert(ipstr);
            }
        }

        freeifaddrs(ifList);
        return ipset;
    }
};

SdkDahua::SdkDahua(const std::string &ip, const std::string &user, const std::string &pwd)
{
    impl_ = std::make_unique<SdkDahuaImpl>();
    impl_->ip_ = ip;
    impl_->user_ = user;
    impl_->pwd_ = pwd;
}

/**
 * @description: 不带IP的，需要手动启动IP搜索后再设置IP，才能start
 * @param user
 * @param pwd
 * @return {}
 */
SdkDahua::SdkDahua(const std::string &user, const std::string &pwd)
{
    impl_ = std::make_unique<SdkDahuaImpl>();
    impl_->user_ = user;
    impl_->pwd_ = pwd;
}

SdkDahua::~SdkDahua() { impl_->stop_talk(); }

void SdkDahua::set_ip(const std::string &ip) { impl_->ip_ = ip; }

/**
 * @description: 设置云台转动
 * @param vx 速度
 * @param vy
 * @return {}
 */
void SdkDahua::set_ptz(double msg_vx, double msg_vy) { impl_->set_ptz(msg_vx, msg_vy); }

/**
 * @description: 视频流数据回调，目前是h264流
 * @return {}
 */
void SdkDahua::set_video_callback(std::function<void(const std::vector<uint8_t> &)> func) { impl_->video_cb_ = func; }

void SdkDahua::set_audio_callback(std::function<void(const std::vector<uint8_t> &)> func) { impl_->audio_cb_ = func; }

void SdkDahua::send_audio(const std::vector<uint8_t> &data) { impl_->send_audio(data); }

/**
 * @description: 开启搜索相机，搜索完毕后，用户需要自行关闭搜索
 * @return {}
 */
void SdkDahua::start_search() { impl_->start_search(); }

void SdkDahua::stop_search() { impl_->stop_search(); }

std::set<std::string> SdkDahua::get_search_result() { return impl_->ip_list_; }

bool SdkDahua::login() { return impl_->login(); }

bool SdkDahua::start_talk() { impl_->start_talk(); }
bool SdkDahua::stop_talk() { impl_->stop_talk(); }

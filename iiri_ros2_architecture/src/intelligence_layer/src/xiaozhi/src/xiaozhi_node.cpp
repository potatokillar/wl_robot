/*
 * @Author: 唐文浩
 * @Date: 2025-04-11
 * @LastEditors: 唐文浩
 * @LastEditTime: 2025-07-04
 * @Description:
 *
 * Copyright (c) 2022-2025 by 西湖大学智能产业研究院, All Rights Reserved.
 */
#include "xiaozhi_node.hpp"

#include <chrono>

// 初始化全局节点智能指针
std::shared_ptr<XiaoZhiNode> g_node_ptr = nullptr;
extern std::shared_ptr<moodycamel::ReaderWriterQueue<std::string>> g_ptr_stt_queue;
// 展开命名空间
using json = nlohmann::json;
// 宏定义
#define OPUS_BUF_SIZE (1024 * 5) /* 60ms的OPUS数据, 5K足够了 */
#define BUFFER_SIZE (1024 * 30)  /* 上传60ms的数据,以441000的采样率,双通道,16bit,最大数据量:44100*2*2*60/1000=10584=10K, 给它3倍 */
#define ENABLE_WRITE_PCM 0
// 定义局部静态变量
// static std::atomic<bool> s_audio_upload_enable = {true};
static std::string s_session_id;
static DeviceState s_device_state = kDeviceStateUnknown;
// static opus_encoder g_opus_encoder;
// static opus_decoder g_opus_decoder;
static unsigned char s_opus_record_buffer[OPUS_BUF_SIZE]; /* 把录音数据编码为OPUS后存在这里 */
static unsigned char s_record_buffer[BUFFER_SIZE];        /* 读取到的录音原始数据 */
static int s_pcm_data_required_size;                      // 转换为opus数据所需的pcm数据的字节数
static unsigned char s_opus_play_buffer[OPUS_BUF_SIZE];   /* 把要播放的OPUS码流存在这里 */
static moodycamel::ReaderWriterQueue<int16_t> s_int16_buffer(32 * 1024);
static std::atomic<bool> s_xiaozhi_enable = {true};      // 是否开启xiaozhi
static std::atomic<bool> s_received_opus_data = {false}; // 最近是否收到了音频文件，用于处理ding音效的播放
static std::atomic<bool> s_dismiss_opus_data = {false};  // 不播放服务端返回的音频数据的标志位
static std::atomic<bool> s_is_answering = {false};       // 服务端正在做出回应
static AudioDataDismissTimer s_audio_dismiss_timer;
bool g_service_sound_enable = true; // 是否播放服务上线、下线音效的标志位
// 各个节点上报的iot当前状态
static std::atomic<u_int16_t> s_current_volume = {0};

/**
 * 生成 UUID
 *
 * 该函数使用 std::random_device 和 std::mt19937 生成一个随机的 UUID。
 * UUID 的格式为 8-4-4-4-12 的 32 位十六进制数字。
 *
 * @return 生成的 UUID 字符串
 */
static std::string generate_uuid()
{
    // 使用静态变量确保 random_device 和 mt19937 只被初始化一次
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, 15);
    std::uniform_int_distribution<> dis2(8, 11);

    std::stringstream ss;
    int i;

    ss << std::hex;

    // 生成 UUID 的各个部分
    for (i = 0; i < 8; i++)
    {
        ss << dis(gen);
    }
    ss << "-";
    for (i = 0; i < 4; i++)
    {
        ss << dis(gen);
    }
    ss << "-";
    ss << dis2(gen);
    for (i = 0; i < 3; i++)
    {
        ss << dis(gen);
    }
    ss << "-";
    ss << dis2(gen);
    for (i = 0; i < 3; i++)
    {
        ss << dis(gen);
    }
    ss << "-";
    for (i = 0; i < 12; i++)
    {
        ss << dis(gen);
    };

    return ss.str();
}

static void set_device_state(DeviceState state) { s_device_state = state; }

std::string IotCmdSetVolume(int volume)
{
    nlohmann::json j;
    j["method"] = "SetVolume";
    j["name"] = "speaker";
    nlohmann::json j_volume;
    j_volume["volume"] = volume;
    j["parameters"] = j_volume;
    return j.dump();
}

void ParseIotJson(nlohmann::json j)
{
    std::string cmd_pub = j.dump();
    g_node_ptr->SendIotCmd(cmd_pub);
}

static void process_hello_json(const char *buffer, size_t size)
{
    g_wakeup.store(false); // 在收到hello消息时，说明聊天流程重置了，所以要把唤醒状态也重置为false
    (void)buffer;
    (void)size;

    json j = json::parse(buffer);
    std::string j_str = j.dump();
    const char *j_c_str = j_str.c_str();
    RCLCPP_DEBUG(g_node_ptr->get_logger(), "process_hello_json() receive content: %s", j_c_str);

    std::string desc =
        R"({
                "session_id":"",
                "type":"iot",
                "update":true,
                "descriptors":[
                    {
                        "name":"speaker",
                        "description":"扬声器",
                        "properties":{
                            "volume":{
                                "description":"当前音量值",
                                "type":"number"
                            }
                        },
                        "methods":{
                            "SetVolume":{
                                "description":"设置音量",
                                "parameters":{
                                    "volume":{
                                        "description":"0到100之间的整数",
                                        "type":"number"
                                    }
                                }
                            }
                        }
                    }
                ]
            })";
    try
    {
        websocket_send_text(desc.data(), desc.size());
        static bool s_hello_json_print = false;
        if (!s_hello_json_print)
        {
            RCLCPP_DEBUG(g_node_ptr->get_logger(), "Send: %s", desc.c_str());
            s_hello_json_print = true;
        }
        else
        {
            RCLCPP_DEBUG(g_node_ptr->get_logger(), "Send hello json!");
        }
    }
    catch (const websocketpp::lib::error_code &e)
    {
        RCLCPP_WARN(g_node_ptr->get_logger(), "Error sending message: %d, (%s)", e.value(), e.message().c_str());
    }

    std::string startString = R"({"session_id":"","type":"listen","state":"start","mode":"auto"})";
    try
    {
        // c->send(hdl, startString, websocketpp::frame::opcode::text);
        websocket_send_text(startString.data(), startString.size());
        RCLCPP_DEBUG(g_node_ptr->get_logger(), "Send: %s", startString.c_str());
    }
    catch (const websocketpp::lib::error_code &e)
    {
        RCLCPP_WARN(g_node_ptr->get_logger(), "Error sending message: %d, (%s)", e.value(), e.message().c_str());
    }

    // 上报当前iot设备的值
    /** @todo 2025.06.18 - 如果中途调节过音量的值，那这个值要更新到每次上报的字段里，但是目前还未实现 */
    nlohmann::json current_iot_state;
    current_iot_state["session_id"] = "";
    current_iot_state["type"] = "iot";
    current_iot_state["update"] = true;
    nlohmann::json states = json::array();
    nlohmann::json speaker_state;
    speaker_state["name"] = "speaker";
    nlohmann::json speaker_param;
    speaker_param["volume"] = s_current_volume.load(); // 把当前的音量值存放进json中，更新到服务端
    speaker_state["state"] = speaker_param;
    states.push_back(speaker_state);
    current_iot_state["states"] = states;
    std::string state = current_iot_state.dump();

    try
    {
        websocket_send_text(state.data(), state.size());
        RCLCPP_DEBUG(g_node_ptr->get_logger(), "Send: %s", state.c_str());
    }
    catch (const websocketpp::lib::error_code &e)
    {
        RCLCPP_WARN(g_node_ptr->get_logger(), "Error sending message: %d, (%s)", e.value(), e.message().c_str());
    }

    g_audio_upload_enable.store(true);
}

static void process_opus_data_downloaded(const char *buffer, size_t size)
{
    // 如果此时不处于唤醒状态，直接丢弃这部分opus音频，不播放
    // if (!g_wakeup.load()) {
    //     return;
    // }
    // 如果抛弃音频数据的标志位置为true，而且正在回应标志位为false
    // 则不做任何事，直接退出
    if (s_dismiss_opus_data.load())
    {
        return;
    }
    s_received_opus_data.store(true);
    memcpy(s_opus_play_buffer, buffer, size);
    int l_pcm_data_size = BUFFER_SIZE;
    // unsigned char l_pcm_data[l_pcm_data_size];
    std::vector<unsigned char> l_pcm_data(l_pcm_data_size);
    // 解码成pcm数据(比特率16000，单声道，S16_LE)，存放到1个公用容器中，有另外的线程负责发送
    int ret = opus2pcm(s_opus_play_buffer, size, l_pcm_data.data(), &l_pcm_data_size);
    if (ret == 0)
    {
        // opus解码为pcm成功
        // /*DEBUG*/ RCLCPP_INFO(g_node_ptr->get_logger(), "ret = %d", ret);
        // std::vector<int16_t> l_data_to_speaker;
        // 处理l_pcm_data大小不是偶数的情况
        if (l_pcm_data_size % 2 != 0)
        {
            l_pcm_data.resize(l_pcm_data_size + 1);
            l_pcm_data[l_pcm_data_size] = 0x00;
            l_pcm_data_size++;
        }
        for (int i = 0; i < l_pcm_data_size; i += 2)
        {
            // 小端序组合两个字节为一个int16_t
            int16_t sample = static_cast<int16_t>(l_pcm_data[i]) | (static_cast<int16_t>(l_pcm_data[i + 1]) << 8);
            // l_data_to_speaker.push_back(sample);
            s_int16_buffer.enqueue(sample);
        }
        // 将音频数据发送给speaker节点让他播放
        // auto msg_to_speaker = interface::msg::AudioDataPackage();
        // msg_to_speaker.data = l_data_to_speaker;
        // _audio_data_pub_ptr->publish(msg_to_speaker);
    }
}

static void send_start_listening_req(ListeningMode mode)
{
    std::string startString = "{\"session_id\":\"" + s_session_id + "\"";

    startString += ",\"type\":\"listen\",\"state\":\"start\"";

    if (mode == kListeningModeAutoStop)
    {
        startString += ",\"mode\":\"auto\"}";
    }
    else if (mode == kListeningModeManualStop)
    {
        startString += ",\"mode\":\"manual\"}";
    }
    else if (mode == kListeningModeAlwaysOn)
    {
        startString += ",\"mode\":\"realtime\"}";
    }

    try
    {
        websocket_send_text(startString.data(), startString.size());
        RCLCPP_DEBUG(g_node_ptr->get_logger(), "Send: %s", startString.c_str());
    }
    catch (const websocketpp::lib::error_code &e)
    {
        RCLCPP_WARN(g_node_ptr->get_logger(), "Error sending message: %d, (%s)", e.value(), e.message().c_str());
    }
}

static void process_other_json(const char *buffer, size_t size)
{
    (void)size;

    RCLCPP_DEBUG(g_node_ptr->get_logger(), "process_other_json(), content: %s", buffer);
    try
    {
        // Parse JSON data
        json j = json::parse(buffer);

        if (!j.contains("type"))
            return;

        if (j["type"] == "tts")
        {
            auto state = j["state"];
            if (state == "start")
            {
                // 下发语音, 可以关闭录音
                if (g_wakeup.load())
                {
                    g_audio_upload_enable.store(false);
                }
                set_device_state(kDeviceStateListening);
                // send_device_state();
            }
            else if (state == "stop")
            {
                // 本次交互结束, 可以继续上传声音
                // 等待一会以免她听到自己的话误以为再次对话
                sleep(1);
                send_start_listening_req(kListeningModeAutoStop);
                set_device_state(kDeviceStateListening);
                // send_device_state();
                g_audio_upload_enable.store(true);
                if (g_wakeup.load())
                {
                    if (s_received_opus_data.load())
                    {
                        g_node_ptr->SendSpeakerWavFile("ding");
                        s_received_opus_data.store(false);
                    }
                }
                g_wakeup.store(false);
            }
            else if (state == "sentence_start")
            {
                auto text = j["text"];
                std::string l_str = text.get<std::string>();
                send_start_listening_req(kListeningModeAutoStop);
                set_device_state(kDeviceStateSpeaking);
                // 如果此时不处于抛弃音频模式，说明是正常对话的状态
                // 这个时候才要改动正在回应标志位
                // 否则的话，也不改动
                if (!s_dismiss_opus_data.load())
                {
                    s_is_answering.store(true);
                }
            }
            else if (state == "sentence_end")
            {
                if (!s_dismiss_opus_data.load())
                {
                    s_is_answering.store(false);
                    s_audio_dismiss_timer.Activate(); // 正常对话状态，1个sentence结束后，刷新一下计时器
                }
            }
        }
        else if (j["type"] == "stt")
        {
            s_audio_dismiss_timer.Activate(); // 刷新计时器
            // 表示服务器端识别到了用户语音, 取出"text", 通知GUI
            auto text = j["text"];
            std::string l_str = text.get<std::string>();
            // 检测唤醒词，设置标志位g_wakeup
            // if (l_str.find("小黑") != std::string::npos) {
            //     g_wakeup.store(true);
            //     g_audio_upload_enable.store(false);  // 唤醒后提前关闭麦克风，等待服务端说话，服务端说完了，再在其他地方重新打开麦克风
            // }
            // 不要唤醒词，持续唤醒
            g_wakeup.store(true);
            g_audio_upload_enable.store(false);
            // 将语音识别结果放入队列
            if (g_wakeup.load())
            {
                g_ptr_stt_queue->enqueue(text);
            }
        }
        else if (j["type"] == "llm")
        {
            auto emotion = j["emotion"];
        }
        else if (j["type"] == "iot")
        {
            // commands字段里面是server端发来的控制命令
            if (j.contains("commands") && j["commands"].is_array())
            {
                auto commands = j["commands"];
                ParseIotJson(commands); // 解析iot字段的命令
            }
        }
    }
    catch (json::parse_error &e)
    {
        RCLCPP_WARN(g_node_ptr->get_logger(), "Failed to parse JSON message: %s", e.what());
    }
    catch (std::exception &e)
    {
        RCLCPP_WARN(g_node_ptr->get_logger(), "Error processing message: %s", e.what());
    }
}

static void process_txt_data_downloaded(const char *buffer, size_t size)
{
    (void)size;

    try
    {
        // Parse the JSON message
        json j = json::parse(buffer);

        // Check if the message matches the expected structure
        if (j.contains("type") && j["type"] == "hello")
        {
            process_hello_json(buffer, size);
        }
        else
        {
            process_other_json(buffer, size);
        }
    }
    catch (json::parse_error &e)
    {
        RCLCPP_WARN(g_node_ptr->get_logger(), "Failed to parse JSON message: %s", e.what());
    }
    catch (std::exception &e)
    {
        RCLCPP_WARN(g_node_ptr->get_logger(), "Error processing message: %s", e.what());
    }
}

AudioDataDismissTimer::AudioDataDismissTimer() { Open(); }

AudioDataDismissTimer::~AudioDataDismissTimer()
{
    Close();
    // 确保计时线程结束
    if (timer_.joinable())
    {
        timer_.join();
    }
}

void AudioDataDismissTimer::Open()
{
    now_.store(duration_);

    if (!valid_.load())
    {
        s_dismiss_opus_data.store(false);

        timer_ = std::thread(&AudioDataDismissTimer::TimerThread, this);
        timer_.detach();
        valid_.store(true);
    }
}

void AudioDataDismissTimer::Close()
{
    std::lock_guard<std::mutex> lock(mtx_);
    valid_.store(false);

    s_dismiss_opus_data.store(true);
}

void AudioDataDismissTimer::TimerThread()
{
    while (now_.load() > 0)
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        // 如果此时正在回应状态，则将计时器刷新至初始状态
        if (s_is_answering.load())
        {
            now_.store(duration_);
        }
        // RCLCPP_DEBUG(g_node_ptr->get_logger(), "waiting for 1 second, and there are %ld second left.", now_.load());
        now_.fetch_sub(1);
    }
    Close();
    // RCLCPP_DEBUG(g_node_ptr->get_logger(), "time out, timer will be destructed immediately.");
}

void AudioDataDismissTimer::Activate()
{
    RCLCPP_DEBUG(g_node_ptr->get_logger(), "activate Timer");
    std::lock_guard<std::mutex> lock(mtx_);
    if (valid_.load())
    {
        now_.store(duration_);
        RCLCPP_DEBUG(g_node_ptr->get_logger(), "Timer refresh!");
    }
    else
    {
        Open();
    }
}

XiaoZhiNode::XiaoZhiNode() : Node("xiaozhi_node")
{
    // 初始化ros通信接口
    // 初始化publisher侧的topic接口
    _audio_data_pub_ptr = this->create_publisher<interface::msg::AudioDataPackage>("/speaker/audio_data", 10);
    _xiaozhi_stt_pub_ptr = this->create_publisher<std_msgs::msg::String>("/xiaozhi/stt", 10);
    _wav_file_pub_ptr = this->create_publisher<std_msgs::msg::String>("/speaker/wav_file", 10);
    _iot_cmd_pub_ptr = this->create_publisher<std_msgs::msg::String>("/xiaozhi/iot_cmd", 10);
    // 初始化subscriber侧的topic接口
    _audio_data_sub_ptr =
        this->create_subscription<interface::msg::AudioDataPackage>("/record/audio_data", 10, [this](const interface::msg::AudioDataPackage::SharedPtr msg)
                                                                    { this->CallbackRecord(msg); });
    // _xiaozhi_cmd_sub_ptr = this->create_subscription<std_msgs::msg::String>("/xiaozhi/cmd", 10, [this](const std_msgs::msg::String::SharedPtr msg) {
    //     this->CallbackXiaoZhiCmd(msg);
    // });
    _current_volume_sub_ptr =
        this->create_subscription<std_msgs::msg::UInt16>("speaker/current_volume", 10, [this](const std_msgs::msg::UInt16::SharedPtr msg)
                                                         { this->CallbackCurrentVolume(msg); });

    _thread_xiaozhi = std::thread(&XiaoZhiNode::LoopXiaoZhiWork, this);
    _thread_publish_audio_data = std::thread(&XiaoZhiNode::LoopSendToSpeaker, this);
    _thread_publish_stt = std::thread(&XiaoZhiNode::LoopSendToASR, this);

    init_opus_encoder(16000, 1, 60, 16000, 1); // 初始化OPUS编码器
    init_opus_decoder(16000, 1, 60, 16000, 1); // 初始化OPUS编码器
    s_pcm_data_required_size = 16000 * 60 / 1000 * 1 * sizeof(opus_int16);

    // 创建1个detach线程，延迟5秒钟之后，根据yaml文件里面的配置，初始化iot各参数，发布控制指令
    this->declare_parameter<int>("init_volume", 0);
    int init_volume = this->get_parameter("init_volume").as_int();
    s_current_volume.store(init_volume);
    std::thread thread_init_volume([init_volume, this]
                                   {
        std::this_thread::sleep_for(std::chrono::milliseconds(5000));
        std::string cmd_pub = IotCmdSetVolume(init_volume);
        RCLCPP_DEBUG(this->get_logger(), "xiaozhi init. cmd_pub = %s", cmd_pub.c_str());
        this->SendIotCmd(cmd_pub); });
    thread_init_volume.detach();
}

void XiaoZhiNode::LoopXiaoZhiWork()
{
    std::this_thread::sleep_for(std::chrono::milliseconds(5000)); // 等待xiaozhi构造函数结束，可以使用节点指针来输出日志
    // 小智模块的代码
    char active_code[20] = "";
    std::string mac = "30:ed:a0:28:24:24";
    std::string uuid = generate_uuid();

    http_data_t http_data;
    http_data.url = "https://api.tenclass.net/xiaozhi/ota/";

    // 替换 http_data.post 中的 uuid
    std::ostringstream post_stream;
    post_stream << R"(
         {
             "uuid":")"
                << uuid << R"(",
             "application": {
                 "name": "xiaozhi_linux_100ask", 
                 "version": "1.0.0"
             },
             "ota": {
             },
             "board": {
                 "type": "100ask_linux_board", 
                 "name": "100ask_imx6ull_board" 
             }
         }
     )";
    http_data.post = post_stream.str();

    // 替换 http_data.headers 中的 Device-Id
    std::ostringstream headers_stream;
    headers_stream << R"(
         {
             "Content-Type": "application/json",
             "Device-Id": ")"
                   << mac << R"(",
             "User-Agent": "weidongshan1",
             "Accept-Language": "zh-CN"
         }
     )";
    http_data.headers = headers_stream.str();

    // 设备激活重试机制
    int activation_attempts = 0;
    while (0 != active_device(&http_data, active_code))
    {
        activation_attempts++;
        RCLCPP_WARN(this->get_logger(), "设备激活失败，尝试次数: %d", activation_attempts);

        if (active_code[0])
        {
            std::string auth_code = "Active-Code: " + std::string(active_code);
            RCLCPP_INFO(this->get_logger(), "激活码: %s", auth_code.c_str());
            set_device_state(kDeviceStateActivating);
        }
        sleep(5);
    }

    RCLCPP_INFO(this->get_logger(), "设备激活成功");
    set_device_state(kDeviceStateIdle);

    websocket_data_t ws_data;

    // 替换 ws_data.headers 中的 Device-Id 和 Client-Id
    std::ostringstream ws_headers_stream;
    ws_headers_stream << R"(
         {
             "Authorization": "Bearer test-token",
             "Protocol-Version": "1",
             "Device-Id": ")"
                      << mac << R"(",
             "Client-Id": ")"
                      << uuid << R"("
         }
     )";
    ws_data.headers = ws_headers_stream.str();

    ws_data.hello = R"(
         {
             "type": "hello",
             "version": 1,
             "transport": "websocket",
             "audio_params": {
                 "format": "opus",
                 "sample_rate": 16000,
                 "channels": 1,
                 "frame_duration": 60
             }
         })";

    ws_data.hostname = "api.tenclass.net";
    ws_data.port = "443";
    ws_data.path = "/xiaozhi/v1/";

    websocket_set_callbacks(process_opus_data_downloaded, process_txt_data_downloaded, &ws_data);
    while (1)
    {
        RCLCPP_INFO(this->get_logger(), "xiaozhi node init successfully");
        websocket_start();
    }
}

void XiaoZhiNode::SendSpeakerWavFile(std::string str)
{
    // 把播放任务传递给speaker节点
    auto filename_to_speaker = std_msgs::msg::String();
    filename_to_speaker.data = str;
    _wav_file_pub_ptr->publish(filename_to_speaker);
}

void XiaoZhiNode::SendIotCmd(std::string str)
{
    auto iod_cmd_pub_content = std_msgs::msg::String();
    iod_cmd_pub_content.data = str;
    _iot_cmd_pub_ptr->publish(iod_cmd_pub_content);
}

void XiaoZhiNode::CallbackRecord(const interface::msg::AudioDataPackage::SharedPtr msg)
{
    // 需要足够的pcm数据，才能编码出足够的opus数据，服务器才会有响应。仿照xiaozhi-linux原来的接口使用方式。
    static int s_record_buffer_offset = 0; // 偏移量计数器

    std::vector<int16_t> l_audio_data;
    if (s_xiaozhi_enable.load())
    {
        l_audio_data = msg->data;
    }
    else
    {
        // l_audio_data(0x00, 256);
        std::vector<int16_t> l_temp(0x00, 256);
        l_audio_data = l_temp;
    }

    int l_audio_data_size = l_audio_data.size();
    int l_pcm_data_size = l_audio_data_size * sizeof(int16_t);
    // /*DEBUG*/ RCLCPP_INFO(this->get_logger(), "receive AudioDataPackage, num of bytes: %d", l_pcm_data_size);
    unsigned char *l_pcm_data = new unsigned char[l_pcm_data_size];
    for (int i = 0; i < l_audio_data_size; ++i)
    {
        int16_t value = l_audio_data[i];
        l_pcm_data[i * 2] = static_cast<unsigned char>(value & 0xFF);
        l_pcm_data[i * 2 + 1] = static_cast<unsigned char>((value >> 8) & 0xFF);
    }
    memcpy(s_record_buffer + s_record_buffer_offset, l_pcm_data, l_pcm_data_size); // 将原始pcm格式的音频数据，拷贝到缓存中
    s_record_buffer_offset += l_pcm_data_size;
    delete[] l_pcm_data;

    int i = 0;
    int pcmsize = 0;
    int opussize = 0;
    if (s_record_buffer_offset >= s_pcm_data_required_size)
    {
#if ENABLE_WRITE_PCM
        std::ofstream opus_file("xiaozhi.opus", std::ios::binary | std::ios::app);
        if (!opus_file.is_open())
        {
            RCLCPP_ERROR(rclcpp::get_logger("CallbackRecord"), "Failed to open xiaozhi.opus for writing");
            return;
        }
#endif
        while (i < s_record_buffer_offset)
        {
            pcmsize = s_record_buffer_offset - i;
            if (pcmsize > s_pcm_data_required_size)
                pcmsize = s_pcm_data_required_size;
            else
            {
                // 如果剩余的数据不足一个Opus包，则保留下来移动到最前面
                memcpy(s_record_buffer, s_record_buffer + i, pcmsize);
                break;
            }
            pcm2opus(s_record_buffer + i, pcmsize, s_opus_record_buffer, &opussize);

            if (opussize)
            {
                // /*DEBUG*/ RCLCPP_INFO(this->get_logger(), "成功转换opus数据，字节数: %d, s_record_buffer_offset: %d", opussize, s_record_buffer_offset);
                if (g_audio_upload_enable.load())
                {
                    websocket_send_binary((const char *)s_opus_record_buffer, opussize);
                    // /*DEBUG*/ RCLCPP_INFO(this->get_logger(), "上传opus数据");
                }
#if ENABLE_WRITE_PCM
                // 将Opus数据写入文件
                opus_file.write(reinterpret_cast<const char *>(s_opus_record_buffer), opussize);
#endif
            }
            i += pcmsize;
        }
#if ENABLE_WRITE_PCM
        // 关闭文件
        opus_file.close();
#endif
        // 重置s_record_buffer_offset
        s_record_buffer_offset -= i;
    }
}

void XiaoZhiNode::CallbackCurrentVolume(const std_msgs::msg::UInt16::SharedPtr msg) { s_current_volume.store(msg->data); }

// void XiaoZhiNode::CallbackXiaoZhiCmd(const std_msgs::msg::String::SharedPtr msg)
// {
//     /*DEBUG*/ RCLCPP_INFO(this->get_logger(), "CallbackXiaoZhiCmd, cmd: %s", msg->data.c_str());
//     if (msg->data == "start") {
//         s_xiaozhi_enable.store(true);
//     } else if (msg->data == "stop") {
//         s_xiaozhi_enable.store(false);
//     }
// }

void XiaoZhiNode::LoopSendToSpeaker()
{
    // std::ofstream file("LoopSendToSpeaker.pcm", std::ios::binary | std::ios::trunc);
    while (1)
    {
        if (s_int16_buffer.size_approx() > 960)
        {
            std::vector<int16_t> l_audio_data(960);
            for (auto it = l_audio_data.begin(); it != l_audio_data.end(); it++)
            {
                int16_t temp;
                bool ret = s_int16_buffer.try_dequeue(temp);
                if (ret == true)
                {
                    *it = temp;
                }
            }
            auto l_package = interface::msg::AudioDataPackage();
            l_package.data = l_audio_data;
            _audio_data_pub_ptr->publish(l_package);
            // RCLCPP_DEBUG(this->get_logger(), "Send AudioDataPackage topic!");
            // // 将这部分音频数据写入到pcm文件里
            // char l_data_array[256];
            // for (unsigned int i = 0; i < l_audio_data.size(); i++) {
            //     int16_t item = l_audio_data.at(i);
            //     int8_t lower_byte = item & 0xFF;
            //     int8_t upper_byte = (item >> 8) & 0xFF;
            //     l_data_array[2 * i] = lower_byte;
            //     l_data_array[2 * i + 1] = upper_byte;
            // }
            // if (file.is_open()) {
            //     file.write(reinterpret_cast<const char*>(l_data_array), 256);
            // }
        }
        // 这里的50 ms和960个int16，是匹配的，如果要修改，2个要一起修改
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

void XiaoZhiNode::LoopSendToASR()
{
    while (1)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        std::string temp;
        if (g_ptr_stt_queue->try_dequeue(temp))
        {
            auto msg = std_msgs::msg::String();
            msg.data = temp;
            _xiaozhi_stt_pub_ptr->publish(msg);
            RCLCPP_DEBUG(g_node_ptr->get_logger(), "publish a /xiaozhi/stt topic message");
        }
    }
}
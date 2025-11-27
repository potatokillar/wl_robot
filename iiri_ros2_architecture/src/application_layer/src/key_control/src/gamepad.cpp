#include "gamepad.hpp"

#include <fcntl.h>
#include <linux/input.h>
#include <linux/joystick.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <string>
#include <vector>

#include "robot_base/cpp_types.hpp"
#include "robot_base/timer_tools.hpp"

using namespace std;
namespace fs = std::filesystem;

// linux 手柄类型定义
constexpr int8_t GAMEPAD_TYPE_BUTTON = 0x01;
constexpr int8_t GAMEPAD_TYPE_AXIS = 0x02;

constexpr int8_t GAMEPAD_BUTTON_A = 0x00;
constexpr int8_t GAMEPAD_BUTTON_B = 0x01;
constexpr int8_t GAMEPAD_BUTTON_X = 0x02;
constexpr int8_t GAMEPAD_BUTTON_Y = 0x03;
constexpr int8_t GAMEPAD_BUTTON_LB = 0x04;
constexpr int8_t GAMEPAD_BUTTON_RB = 0x05;
constexpr int8_t GAMEPAD_BUTTON_BACK = 0x06;
constexpr int8_t GAMEPAD_BUTTON_START = 0x07;
constexpr int8_t GAMEPAD_BUTTON_HOME = 0x08;  //  未用
constexpr int8_t GAMEPAD_BUTTON_LO = 0x09;    /* 左摇杆按键 */
constexpr int8_t GAMEPAD_BUTTON_RO = 0x0a;    /* 右摇杆按键 */

// XBOX和ps下面的手柄定义略有不同，这里使用xbox的
constexpr int8_t GAMEPAD_STICK_LX = 0x00;       // 左摇杆横轴，左-32767，右32767
constexpr int8_t GAMEPAD_STICK_LY = 0x01;       // 左摇杆纵轴，上-32767，下32767
constexpr int8_t GAMEPAD_STICK_LT = 0x02;       // LT 弹起-32767，按下32767
constexpr int8_t GAMEPAD_STICK_RX = 0x03;       // 右摇杆横轴，左-32767，右32767
constexpr int8_t GAMEPAD_STICK_RY = 0x04;       // 右摇杆纵轴，上-32767，下32767
constexpr int8_t GAMEPAD_STICK_RT = 0x05;       // RT 参数同LT
constexpr int8_t GAMEPAD_BUTTON_ARROWX = 0x06;  // 左方向键横轴，左-32767，右32767
constexpr int8_t GAMEPAD_BUTTON_ARROWY = 0x07;  // 左方向键纵轴，上-32767，下32767

constexpr int16_t GAMEPAD_STICK_VAL_UP = -32767;
constexpr int16_t GAMEPAD_STICK_VAL_DOWN = 32767;
constexpr int16_t GAMEPAD_STICK_VAL_LEFT = -32767;
constexpr int16_t GAMEPAD_STICK_VAL_RIGHT = 32767;
constexpr int16_t GAMEPAD_STICK_VAL_MIN = -32767;
constexpr int16_t GAMEPAD_STICK_VAL_MAX = 32767;
constexpr int16_t GAMEPAD_STICK_VAL_MID = 0x00;

// 由Linux传递上来的键值
struct InputKey
{
    // 按键，按下为1，弹起为0
    int16_t btnA{0};
    int16_t btnB{0};
    int16_t btnX{0};
    int16_t btnY{0};
    int16_t btnLb{0};
    int16_t btnRb{0};
    int16_t btnBack{0};
    int16_t btnStart{0};
    int16_t btnLpress{0};     // 左摇杆按键
    int16_t btnRpress{0};     // 右摇杆按键
    int16_t btnUpDown{0};     // 左十字方向按键上下，上-32767，下32767
    int16_t btnLeftRight{0};  // 左十字方向按键左右，左-32767，右32767

    int16_t stickLx{0};                      // 左摇杆x轴，左-32767，右32767
    int16_t stickLy{0};                      // 左摇杆y轴，上-32767，下32767
    int16_t stickRx{0};                      // 右摇杆x轴，参数同上
    int16_t stickRy{0};                      // 右摇杆y轴
    int16_t stickLt{GAMEPAD_STICK_VAL_MIN};  // LT 弹起-32767，按下32767
    int16_t stickRt{GAMEPAD_STICK_VAL_MIN};  // RT 参数同LT
};

// 手柄类型，不同的手柄映射会不同
enum class GamepadType
{
    xbox360,
    xboxOne,
    other,
};

// 手柄设备
class GamepadDev
{
public:
    /**
     * @description: 检测手柄状态
     * @param
     * @return {} 手柄可用则返回true
     */
    bool IsOk()
    {
        // 判断文件是否存在
        if (access(devPath_.c_str(), F_OK) != 0) {
            return false;
        }

        if ((fd_ < 0) /*|| (evtFd_ < 0)*/) {
            return false;
        }

        return true;
    }

    void Open(const std::string& devName)
    {
        devPath_ = devName;
        fd_ = open(devName.c_str(), O_RDONLY);
        if (fd_ < 0) {
            RCLCPP_ERROR(rclcpp::get_logger("gamepad"), "open failed");
            return;
        }
        TimerTools::SleepForMs(500);
        SaveInfo(devName);
        if (type_ != GamepadType::other) {
            // if (OpenFF(evtPath_) == false) {
            //     fd_ = -1;
            //     return;
            // }

            RCLCPP_INFO(rclcpp::get_logger("gamepad"), "Open success: %s !!!", devName.c_str());
            isOpen_ = true;
            thread_ = std::thread([this]() { this->Run(); });
        }
    }
    bool IsOpen() const { return isOpen_; }

    void Close()
    {
        StopFF();
        isOpen_ = false;
        if (thread_.joinable()) {
            thread_.join();
        }
    }

    /**
     * @description: 开启震动
     * @return {}
     */
    void StartFF()
    {
        if ((isOpen_ == false) || (type_ == GamepadType::other)) {
            return;
        }
        struct input_event play;
        bzero(&play, sizeof(struct input_event));
        play.type = EV_FF;
        play.code = effect_.id;
        play.value = 1;

        auto ret = write(evtFd_, (const void*)&play, sizeof(play));
        if (ret < 0) {
            RCLCPP_WARN(rclcpp::get_logger("gamepad"), "StartFF failed");
            evtFd_ = -1;
        }
    }

    /**
     * @description: 读取
     * @param ms 不是超时，本接口是非阻塞的，该值表示，上一次读取超过ms，则表示无数据，返回nullopt
     * @return {}
     */
    std::optional<GamepadData> Read(u32 ms = 1500)
    {
        if (isOpen_ == false) {
            return std::nullopt;
        }

        if (TimerTools::GetNowTickMs() - last_read_time_ > ms) {
            return std::nullopt;
        }
        lock_guard<mutex> lock(mutex_);
        return xboxToUser(key_);
    }

private:
    /**
     * @description: 由于对外接口的Read不允许阻塞，同时需要判断手柄可用或者无数据，因此需要单独一个线程
     * @return {}
     */
    void Run()
    {
        while (isOpen_) {
            if (fd_ < 0) {
                TimerTools::SleepForMs(200);
                continue;
            }

            struct js_event js;
            if (read(fd_, &js, sizeof(struct js_event)) >= 0) {
                lock_guard<mutex> lock(mutex_);
                if (type_ == GamepadType::xbox360) {
                    key_ = xbox360ToKey(key_, js);
                } else if (type_ == GamepadType::xboxOne) {
                    key_ = xboxOneToKey(key_, js);
                }
                last_read_time_ = TimerTools::GetNowTickMs();
            } else {
                RCLCPP_ERROR(rclcpp::get_logger("gamepad"), "read error %s", strerror(errno));
                fd_ = -1;
            }
        }
    }

    /**
     * @description: 保存手柄的一些必要信息，就是event和手柄类型
     * @param devName
     * @return {}
     */
    void SaveInfo(const std::string& devName)
    {
        std::ifstream ifs("/proc/bus/input/devices");
        if (ifs.is_open() == false) {
            return;
        }

        // 对设备名进行分割
        auto n = devName.rfind("js");
        if (n == string::npos) {
            return;
        }
        auto preDevName = devName.substr(0, n);
        auto jsDevName = devName.substr(n, devName.size() - n);  // 取最后的几个字
        // cout << "js " << jsDevName << endl;
        std::string line;
        std::string joyName;  // joystick的名字
        while (getline(ifs, line)) {
            // 至少得能容纳包含Handlers=那个字符串的长度
            if (line.size() < 12) {
                continue;
            }

            // 记录设备名字，每次轮询到新设备会更新
            if (line.compare(3, 5, "Name=") == 0) {
                joyName = line;
            }
            // 查找手柄对应的event
            if ((line.compare(3, 9, "Handlers=") == 0) && (line.find(jsDevName) != string::npos)) {
                type_ = GetGamepadType(joyName);  // 获取设备类型

                // 获取event的序号
                auto p1 = line.find("event");
                auto p2 = line.find(" ", p1);
                line = line.substr(p1, p2 - p1);
                evtPath_ = preDevName + line;
                break;
            }
        }
    }

    /**
     * @description: 获取手柄的类型
     * @param &name 从/proc/bus/input/devices中得到的名字
     * @return {} 无法匹配时，返回other类型
     */
    GamepadType GetGamepadType(const std::string& name)
    {
        GamepadType type = GamepadType::other;
        if (name == "N: Name=\"Microsoft X-Box 360 pad\"") {
            // 北通阿修罗2(有线+无线)
            type = GamepadType::xbox360;
        } else if (name == "N: Name=\"Xbox Wireless Controller\"") {
            // xboxOne2020 无线连接时，是xboxOne，不过目前不再使用
            // 雷神G50S，蓝牙连接，是xbox360
            type = GamepadType::xbox360;
        } else if (name == "N: Name=\"GamepadX\"") {
            // 雷神G50
            type = GamepadType::xbox360;
        } else if (name == "N: Name=\"BEITONG  BEITONG A1S2 BFM GAMEPAD \"") {
            // 北通阿修罗2pro，新款
            type = GamepadType::xboxOne;
        } else if (name == "N: Name=\"Microsoft Xbox Series S|X Controller\"") {
            // xboxOne2020 有线连接时
            type = GamepadType::xbox360;
        } else if (name == "N: Name=\"Microsoft X-Box One S pad\"") {
            type = GamepadType::xbox360;
        }

        return type;
    }

    /**
     * @description: 打开对应的事件设备
     * @param &devName 事件设备名
     * @return {}
     */
    bool OpenFF(const std::string& evtPath)
    {
        // todo虚拟机未登录，远程非sudo启动时，权限拒绝无法打开
        evtFd_ = open(evtPath.c_str(), O_RDWR);
        if (evtFd_ == -1) {
            RCLCPP_WARN(rclcpp::get_logger("gamepad"), "OpenFF failed: %s, %s", evtPath.c_str(), strerror(errno));
            return false;
        }
        return InitFF();  // 初始化震动
    }
    /**
     * @description: 初始化震动有关参数
     * https://www.kernel.org/doc/html/latest/input/ff.html
     * https://github.com/flosse/linuxconsole/blob/master/utils/fftest.c
     * @return {}
     */
    bool InitFF()
    {
        if (evtFd_ == -1) {
            return false;
        }

        memset(&effect_, 0, sizeof(effect_));
        effect_.type = FF_PERIODIC;
        effect_.id = -1;
        effect_.u.periodic.waveform = FF_SINE;
        effect_.u.periodic.period = 100;       /* 0.1 second */
        effect_.u.periodic.magnitude = 0x7fff; /* 0.5 * Maximum magnitude */
        effect_.u.periodic.offset = 0;
        effect_.u.periodic.phase = 0;
        effect_.direction = 0x4000; /* Along X axis */
        effect_.u.periodic.envelope.attack_length = 1000;
        effect_.u.periodic.envelope.attack_level = 0x7fff;
        effect_.u.periodic.envelope.fade_length = 1000;
        effect_.u.periodic.envelope.fade_level = 0x7fff;
        effect_.trigger.button = 0;
        effect_.trigger.interval = 0;
        effect_.replay.length = 500; /* 0.5 seconds */
        effect_.replay.delay = 1000;

        // 设置震动效果
        if (ioctl(evtFd_, EVIOCSFF, &effect_) == -1) {
            RCLCPP_WARN(rclcpp::get_logger("gamepad"), "InitFF failed");

            evtFd_ = -1;  // 失败需要复位
            return false;
        }
        return true;
    }

    /**
     * @description: 关闭震动
     * @return {}
     */
    void StopFF()
    {
        if (evtFd_ == -1) {
            return;
        }
        struct input_event play;
        bzero(&play, sizeof(struct input_event));
        play.type = EV_FF;
        play.code = effect_.id;
        play.value = 0;

        if (write(evtFd_, (const void*)&play, sizeof(play)) < 0) {
            RCLCPP_WARN(rclcpp::get_logger("gamepad"), "StopFF failed");
            evtFd_ = -1;
        }
    }

    /**
     * @description: xbox360的映射
     * @param &js
     * @return {}
     */
    const InputKey& xbox360ToKey(InputKey& key, const struct js_event& js)
    {
        if (js.type == JS_EVENT_BUTTON) {
            switch (js.number) {
                case GAMEPAD_BUTTON_A:
                    key.btnA = js.value;
                    break;

                case GAMEPAD_BUTTON_B:
                    key.btnB = js.value;
                    break;

                case GAMEPAD_BUTTON_X:
                    key.btnX = js.value;
                    break;

                case GAMEPAD_BUTTON_Y:
                    key.btnY = js.value;
                    break;

                case GAMEPAD_BUTTON_LB:
                    key.btnLb = js.value;
                    break;

                case GAMEPAD_BUTTON_RB:
                    key.btnRb = js.value;
                    break;

                case GAMEPAD_BUTTON_START:
                    key.btnStart = js.value;
                    break;

                case GAMEPAD_BUTTON_BACK:
                    key.btnBack = js.value;
                    break;

                case GAMEPAD_BUTTON_HOME:
                    break;

                case GAMEPAD_BUTTON_LO:
                    key.btnLpress = js.value;
                    break;

                case GAMEPAD_BUTTON_RO:
                    key.btnRpress = js.value;
                    break;

                default:
                    break;
            }
        } else if (js.type == JS_EVENT_AXIS) {
            switch (js.number) {
                case GAMEPAD_STICK_LX:
                    key.stickLx = js.value;
                    break;

                case GAMEPAD_STICK_LY:
                    key.stickLy = js.value;
                    break;

                case GAMEPAD_STICK_RX:
                    key.stickRx = js.value;
                    break;

                case GAMEPAD_STICK_RY:
                    key.stickRy = js.value;
                    break;

                case GAMEPAD_STICK_LT:
                    key.stickLt = js.value;
                    break;

                case GAMEPAD_STICK_RT:
                    key.stickRt = js.value;
                    break;

                case GAMEPAD_BUTTON_ARROWX:
                    key.btnLeftRight = js.value;
                    break;

                case GAMEPAD_BUTTON_ARROWY:
                    key.btnUpDown = js.value;
                    break;

                default:
                    break;
            }
        }
        return key;
    }

    /**
     * @description: xboxOne2020系列手柄映射
     * @param &js
     * @return {}
     */
    const InputKey& xboxOneToKey(InputKey& key, const struct js_event& js)
    {
        if (js.type == JS_EVENT_BUTTON) {
            switch (js.number) {
                case 0:
                    key.btnA = js.value;
                    break;

                case 1:
                    key.btnB = js.value;
                    break;

                case 3:
                    key.btnX = js.value;
                    break;

                case 4:
                    key.btnY = js.value;
                    break;

                case 6:
                    key.btnLb = js.value;
                    break;

                case 7:
                    key.btnRb = js.value;
                    break;

                case 11:
                    key.btnStart = js.value;
                    break;

                case 10:
                    key.btnBack = js.value;
                    break;

                case 13:
                    key.btnLpress = js.value;
                    break;

                case 14:
                    key.btnRpress = js.value;
                    break;

                default:
                    break;
            }
        } else if (js.type == JS_EVENT_AXIS) {
            switch (js.number) {
                case 0:
                    key.stickLx = js.value;
                    break;

                case 1:
                    key.stickLy = js.value;
                    break;

                case 2:
                    key.stickRx = js.value;
                    break;

                case 3:
                    key.stickRy = js.value;
                    break;

                case 5:
                    key.stickLt = js.value;
                    break;

                case 4:
                    key.stickRt = js.value;
                    break;

                    // 左十字方向按键左右，左-32767，右32767
                case 6:
                    key.btnLeftRight = js.value;
                    break;

                // 左十字方向按键上下，上-32767，下32767
                case 7:
                    key.btnUpDown = js.value;
                    break;

                default:
                    break;
            }
        }
        return key;
    }

    /**
     * @description: 将具体的手柄键值修改成内部需要的格式
     * @param &in
     * @return {}
     */
    GamepadData xboxToUser(const InputKey& in)
    {
        GamepadData out;

        out.btnA = in.btnA;
        out.btnB = in.btnB;
        out.btnX = in.btnX;
        out.btnY = in.btnY;
        out.btnLb = in.btnLb;
        out.btnRb = in.btnRb;
        out.btnBack = in.btnBack;
        out.btnStart = in.btnStart;
        out.btnLpress = in.btnLpress;
        out.btnRpress = in.btnRpress;

        if (in.btnUpDown < 0) {
            out.btnUp = 1;
        } else if (in.btnUpDown > 0) {
            out.btnDown = 1;
        }

        if (in.btnLeftRight < 0) {
            out.btnLeft = 1;
        } else if (in.btnLeftRight > 0) {
            out.btnRight = 1;
        }

        out.stickLx = in.stickLx / 32767.0;
        out.stickLy = in.stickLy / 32767.0;
        out.stickRx = in.stickRx / 32767.0;
        out.stickRy = in.stickRy / 32767.0;

        out.stickLt = (in.stickLt / 32767.0 + 1) / 2;
        out.stickRt = (in.stickRt / 32767.0 + 1) / 2;

        return out;
    }

private:
    bool isOpen_{false};

    int fd_{-1};
    std::string devPath_;
    GamepadType type_{GamepadType::other};

    int evtFd_{-1};  // 震动相关
    struct ff_effect effect_;
    std::string evtPath_;

    InputKey key_;
    std::thread thread_;
    std::mutex mutex_;
    u64 last_read_time_{0};
};

/**
 * @description: 查找所有的手柄设备号
 * @return {}
 */
std::vector<std::string> SearchGamepadDev()
{
    std::vector<std::string> devs;

    // 定义要查找的目录路径
    fs::path directoryPath = "/dev/input";
    try {
        // 使用 directory_iterator 遍历目录中的文件和子目录
        for (const auto& entry : fs::directory_iterator(directoryPath)) {
            // 是否为字符文件
            if (entry.is_character_file()) {
                // 如果文件名以 "js" 开头，输出文件名
                if (entry.path().filename().string().compare(0, 2, "js") == 0) {
                    auto name = fs::absolute(entry.path()).string();
                    devs.push_back(name);
                    // LOG_DEBUG("SearchGamepadDev: {}", name);
                    RCLCPP_DEBUG(rclcpp::get_logger("gamepad"), "SearchGamepadDev: %s", name.c_str());
                }
            }
        }
    } catch (const fs::filesystem_error& e) {
        //  LOG_WARN("SearchGamepadDev Error: {}", e.what());
    }
    return devs;
}

static GamepadDev gampedDev;
std::mutex gampedDevMutex;

std::optional<GamepadData> GamepadRead(const std::string& devName)
{
    std::lock_guard<std::mutex> lock(gampedDevMutex);
    // 若设备拔出，则删除设备
    if (gampedDev.IsOk() == false) {
        gampedDev.Close();
    }
    if (gampedDev.IsOpen()) {
        return gampedDev.Read();
    }

    for (auto const& dev : SearchGamepadDev()) {
        if (devName.empty()) {
            gampedDev.Open(dev);
            // gampedDev.StartFF();
            return gampedDev.Read();
        } else if (dev == devName) {
            gampedDev.Open(devName);
            // gampedDev.StartFF();
            return gampedDev.Read();
        }
    }

    return {};
}
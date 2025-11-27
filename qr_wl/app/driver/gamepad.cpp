#include "gamepad.hpp"

#include <fcntl.h>
#include <linux/input.h>
#include <linux/joystick.h>
#include <unistd.h>

#include <cstring>
#include <filesystem>
#include <fstream>
#include <sstream>

#include "baseline.hpp"
#include "robotMedia.hpp"
#include "robotState.hpp"

using namespace std;
namespace fs = std::filesystem;

Gamepad::Gamepad(const string &devPath) : devPath_(devPath)
{
    fd = open(devPath_.c_str(), O_RDONLY | O_NONBLOCK);
    if (fd < 0) {
        return;
    }
    SaveInfo(devPath_);
    if (gamepadType_ != GamepadType::other) {
        OpenFF(evtPath_);
    }

    isOpen_ = true;
}

Gamepad::~Gamepad()
{
    isOpen_ = false;

    if (fd != -1) {
        close(fd);
    }
    if (evtFd_ != -1) {
        close(evtFd_);
    }
}

Gamepad::Gamepad(Gamepad &&other) noexcept
{
    swap(other);
    other.fd = -1;
    other.evtFd_ = -1;
}

Gamepad &Gamepad::operator=(Gamepad &&rhs) noexcept
{
    if (this != &rhs) {
        swap(rhs);
        rhs.fd = -1;
        rhs.evtFd_ = -1;
    }
    return *this;
}

void Gamepad::swap(const Gamepad &other)
{
    fd = other.fd;
    evtFd_ = other.evtFd_;
    key = other.key;
    effect_ = other.effect_;
    isOpen_ = other.isOpen_;
    gamepadType_ = other.gamepadType_;
    devPath_ = other.devPath_;
}

/**
 * @description: 打开一个手柄设备
 * @param &devName
 * @return {} 成功打开返回true
 */
bool Gamepad::IsOpen() const { return isOpen_; }

/**
 * @description: 检测手柄状态
 * @param
 * @return {} 手柄可用则返回true
 */
bool Gamepad::IsOk()
{
    if (access(devPath_.c_str(), F_OK) != 0) {
        return false;
    }
    return true;
}

/**
 * @description: xbox2021的映射
 * @param &js
 * @return {}
 */
const InputKey &Gamepad::xbox360ToKey(InputKey &key, const struct js_event &js)
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
const InputKey &Gamepad::xboxOneToKey(InputKey &key, const struct js_event &js)
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

            case 6:
                key.btnLeftRight = js.value;
                break;

            case 7:
                key.btnUpDown = js.value;
                break;

            default:
                break;
        }
    }
    return key;
}

const InputKey &Gamepad::Read()
{
    struct js_event js;
    fd_set set;

    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 20000;

    FD_ZERO(&set);
    FD_SET(fd, &set);

    if (select(fd + 1, &set, NULL, NULL, &tv)) {
        if (read(fd, &js, sizeof(struct js_event)) < 0) {
            return key;
        }
        if (gamepadType_ == GamepadType::xbox360) {
            key = xbox360ToKey(key, js);
        } else if (gamepadType_ == GamepadType::xboxOne) {
            key = xboxOneToKey(key, js);
        }
    }

    return key;
}

void Gamepad::InitFF()
{
    if (evtFd_ == -1) {
        return;
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
        LOG_WARN("gamepad set ff failed");
    }

#if 0
    // 添加停止，2秒后停止
    SoftTimerSet[1].AddFunc(
        "joyFFstop",
        2000,
        [this]() { this->StopFF(); },
        false);
#endif
}

GamepadType Gamepad::GetGamepadType(const std::string &name)
{
    GamepadType type = GamepadType::other;
    if (name == "N: Name=\"Microsoft X-Box 360 pad\"") {
        type = GamepadType::xbox360;
    } else if (name == "N: Name=\"Xbox Wireless Controller\"") {
        type = GamepadType::xboxOne;
    } else if (name == "N: Name=\"GamepadX\"") {
        type = GamepadType::xbox360;
    } else if (name == "N: Name=\"BEITONG  BEITONG A1S2 BFM GAMEPAD \"") {
        type = GamepadType::xboxOne;
    } else if (name == "N: Name=\"Microsoft X-Box One S pad\"") {
        type = GamepadType::xbox360;
    } else if (name == "N: Name=\"Microsoft Xbox Series S|X Controller\"") {
        type = GamepadType::xbox360;
    }

    // LOG_DEBUG("gamepad type: {}", Enum2Num(type));
    return type;
}

void Gamepad::OpenFF(const std::string &evtPath)
{
    evtFd_ = open(evtPath.c_str(), O_RDWR);
    if (evtFd_ == -1) {
        LOG_WARN("gamepadff open failed: {}, {}", evtPath, strerror(errno));
        return;
    }
    InitFF();  // 初始化震动
}

void Gamepad::SaveInfo(const std::string &devName)
{
    std::ifstream ifs("/proc/bus/input/devices");
    if (ifs.is_open() == false) {
        return;
    }

    auto n = devName.rfind("js");
    if (n == string::npos) {
        return;
    }
    auto preDevName = devName.substr(0, n);
    auto jsDevName = devName.substr(n, devName.size() - n);

    std::string line;
    std::string joyName;
    while (getline(ifs, line)) {
        if (line.size() < 12) {
            continue;
        }

        if (line.compare(3, 5, "Name=") == 0) {
            joyName = line;
        }

        if ((line.compare(3, 9, "Handlers=") == 0) && (line.find(jsDevName) != string::npos)) {
            gamepadType_ = GetGamepadType(joyName);

            auto p1 = line.find("event");
            auto p2 = line.find(" ", p1);
            line = line.substr(p1, p2 - p1);
            evtPath_ = preDevName + line;
            break;
        }
    }
}

/**
 * @description: 开启震动
 * @return {}
 */
void Gamepad::StartFF()
{
    struct input_event play;
    bzero(&play, sizeof(struct input_event));
    play.type = EV_FF;
    play.code = effect_.id;
    play.value = 1;

    auto ret = write(evtFd_, (const void *)&play, sizeof(play));
    (void)ret;
}

/**
 * @description: 关闭震动
 * @return {}
 */
void Gamepad::StopFF()
{
    struct input_event play;
    bzero(&play, sizeof(struct input_event));
    play.type = EV_FF;
    play.code = effect_.id;
    play.value = 0;

    // 会有乱码？
    auto ret = write(evtFd_, (const void *)&play, sizeof(play));
    (void)ret;
}

GamepadNode::GamepadNode()
{
    thread_ = std::thread(&GamepadNode::Loop, this);
    GamepadCheck();  // 立刻进行一次检测
    SoftTimerSet[0].AddFunc("joyCheck", 2000, [this]() { this->GamepadCheck(); });
}

/**
 * @description: 查找所有的手柄设备号
 * @return {}
 */
std::vector<std::string> GamepadNode::SearchGamepadDev()
{
    std::vector<std::string> devs;

    // 定义要查找的目录路径
    fs::path directoryPath = "/dev/input";
    try {
        // 使用 directory_iterator 遍历目录中的文件和子目录
        for (const auto &entry : fs::directory_iterator(directoryPath)) {
            // 是否为字符文件
            if (entry.is_character_file()) {
                // 如果文件名以 "js" 开头，输出文件名
                if (entry.path().filename().string().compare(0, 2, "js") == 0) {
                    auto name = fs::absolute(entry.path()).string();
                    devs.push_back(name);
                    // LOG_DEBUG("SearchGamepadDev: {}", name);
                }
            }
        }
    } catch (const fs::filesystem_error &e) {
        LOG_WARN("SearchGamepadDev Error: {}", e.what());
    }
    return devs;
}

/**
 * @description: 检查joystick的通断
 * @return {}
 */
void GamepadNode::GamepadCheck()
{
    if (gamepad.size() <= 0) {
        for (auto const &dev : SearchGamepadDev()) {
            Gamepad joystick(dev);
            if ((joystick.IsOpen() == true) && (joystick.GetGamepadType() != GamepadType::other)) {
                joystick.StartFF();  // 震动一次
                LOG_INFO(LOG_GREEN + "find gamepad: {}, sdk pause!" + LOG_RESET, joystick.GetGamepadName());
                gamepad.push_back(std::move(joystick));
                MsgTrySend("bs::gamepadFind", true);
                break;
            } else {
                if (isPlay_ == false) {
                    AddMediaPackage(MediaTaskPackage("pleaseConnectJoystick"));
                    isPlay_ = true;
                }
            }
        }
    } else {
        // 检测拔出
        if (gamepad[0].IsOk() == false) {
            LOG_WARN("notice! gamepad missing");
            isPlay_ = false;
            gamepad.clear();
            MsgTrySend("bs::gamepadFind", false);
        }
    }
}

void GamepadNode::Loop()
{
    MsgSend msgGamepad{"bs::gamepad"};
    while (1) {
        if (gamepad.empty() == false) {
            auto key = gamepad[0].Read();  // 该函数是阻塞的
            msg::bs::gamepad_cmd keyUser = xboxToUser(key);
            msgGamepad.TrySend(keyUser);
        }
        TimerTools::SleepForMs(1);  // 释放调度
    }
}

/**
 * @description: 将具体的手柄键值修改成内部需要的格式
 * @param &in
 * @return {}
 */
msg::bs::gamepad_cmd GamepadNode::xboxToUser(const InputKey &in)
{
    msg::bs::gamepad_cmd out;

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

#if 0
    printf("gamepad, A:%d,B:%d,X:%d,Y:%d,LB:%d,RB:%d,BACK:%d,START:%d,LP:%d,RP:%d\n",
           in.btnA,
           in.btnB,
           in.btnX,
           in.btnY,
           in.btnLb,
           in.btnRb,
           in.btnBack,
           in.btnStart,
           in.btnLpress,
           in.btnRpress);
    printf("gamepad, upDown:%d, LeftRight:%d, lx:%d, ly:%d, rx:%d, ry:%d, lt:%d, rt:%d\n",
           in.btnUpDown,
           in.btnLeftRight,
           in.stickLx,
           in.stickLy,
           in.stickRx,
           in.stickRy,
           in.stickLt,
           in.stickRt);
#endif
    return out;
}

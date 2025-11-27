/****************************************************************
 * 功能：gamepad库
 * author  邹应龙
 * create  2021-07-15 新建文件
 * modify  2021-07-15
 * **************************************************************/
#ifndef GAMEPAD_HPP
#define GAMEPAD_HPP

#include <linux/input.h>

#include <cstdint>
#include <optional>
#include <thread>
#include <vector>

#include "baseline.hpp"

namespace msg
{
/// 手柄发出去的信息
struct gamepad_cmd
{
    // 按键，按下为1，弹起为0
    int8_t btnA{0};
    int8_t btnB{0};
    int8_t btnX{0};
    int8_t btnY{0};
    int8_t btnLb{0};
    int8_t btnRb{0};
    int8_t btnBack{0};
    int8_t btnStart{0};
    int8_t btnLpress{0};  // 左摇杆按键
    int8_t btnRpress{0};  // 右摇杆按键
    int8_t btnUp{0};
    int8_t btnDown{0};
    int8_t btnLeft{0};
    int8_t btnRight{0};

    double stickLx{0};  // 左摇杆x轴，左-1，右+1
    double stickLy{0};  // 左摇杆y轴，上-1，下+1
    double stickRx{0};  // 右摇杆x轴
    double stickRy{0};  // 右摇杆y轴
    double stickLt{0};  // LT 弹起0，按下+1
    double stickRt{0};  // RT 参数同LT
};
}  // namespace msg

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
// 需配合xpadneo驱动
enum class GamepadType
{
    xbox360,
    xboxOne,
    other,
};

// 手柄类
class Gamepad
{
public:
    Gamepad(const std::string& devName);
    virtual ~Gamepad();
    Gamepad(const Gamepad& other) = delete;  // 涉及到打开关闭设备，因此不能拷贝
    Gamepad& operator=(const Gamepad& rhs) = delete;
    Gamepad(Gamepad&& other) noexcept;  // 但可以移动
    Gamepad& operator=(Gamepad&& rhs) noexcept;

    bool IsOpen() const;
    const InputKey& Read();
    bool IsOk();

    void StartFF();
    void StopFF();

    void swap(const Gamepad& other);
    GamepadType GetGamepadType() { return gamepadType_; }
    std::string GetGamepadName() { return devPath_; }

private:
    InputKey key;
    int fd = -1;
    int evtFd_ = -1;
    struct ff_effect effect_;
    bool isOpen_ = false;
    std::string devPath_;      // 手柄驱动文件完整路径
    std::string evtPath_;      // 手柄事件文件完整路径
    GamepadType gamepadType_;  // 该手柄的类型

    void OpenFF(const std::string& devName);
    void InitFF();
    const InputKey& xbox360ToKey(InputKey& key, const struct js_event& js);
    const InputKey& xboxOneToKey(InputKey& key, const struct js_event& js);
    GamepadType GetGamepadType(const std::string& name);

    void SaveInfo(const std::string& devName);
};

class GamepadNode : public Singleton<GamepadNode>
{
public:
    GamepadNode();
    template <typename T>
    std::optional<msg::bs::gamepad_cmd> GetKey(T hash)
    {
        return MsgTryRecv<msg::bs::gamepad_cmd>("bs::gamepad", hash);
    }

private:
    std::vector<std::string> SearchGamepadDev();
    void GamepadCheck();
    void Loop() override;
    msg::bs::gamepad_cmd xboxToUser(const InputKey& in);
    std::vector<Gamepad> gamepad;  // 理论上可以支持多个设备，使用指针是让其适当的调用析构
    bool isPlay_{false};
};

inline GamepadNode& GetGamepadNode() { return GamepadNode::GetInstance(); }

#endif
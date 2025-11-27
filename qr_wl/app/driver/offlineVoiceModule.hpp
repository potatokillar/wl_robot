
#include <queue>
#include <vector>

#include "baseline.hpp"
#include "uart.hpp"

class voiceNode final : public Singleton<voiceNode>
{
public:
    void Init() override;
    void Loop() override;
    void ErrorDetect();
    ~voiceNode();

    enum class voiceMessage
    {
        CmdNull = 0,
        CmdNameCall = 1,      // 小黑小黑
        CmdStandUp = 2,       // 起立，站立
        CmdLieDown = 3,       // 趴下
        CmdMarchInPlace = 4,  // 原地踏步
        CmdDance = 5,         // 跳舞
        CmdTurnRight = 6,     // 右转，小黑向右转
        CmdTurnLeft = 7,      // 左转，小黑向左转
        CmdMoveBackward = 8,  // 倒退，小黑向后走
        CmdMoveForward = 9,   // 前进、行走、小黑向前走
        CmdMoveLeft = 10,     // 小黑向左平移
        CmdMoveRight = 11,    // 小黑向右平移
        CmdStandTaller = 12,  // 站高一点
        CmdCrouchLower = 13,  // 趴低一点
        CmdStop = 14,         // 小黑停
        Shutdown = 15,        // 关机
    };

private:
    bool isOk_{false};
    std::unique_ptr<Uart> uart_;
    std::pair<bool, voiceMessage> ParseData(std::queue<uint8_t>& q);
    std::queue<uint8_t> uartData_;  // 原始串口数据队列
};

inline voiceNode& GetVoice() { return voiceNode::GetInstance(); }

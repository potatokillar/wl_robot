#include "offlineVoiceModule.hpp"

#include "baseline.hpp"
#include "deviceParam.hpp"

voiceNode::~voiceNode() { isOk_ = false; }

void voiceNode::Init()
{
    isOk_ = false;
    if (thread_.joinable()) {
        thread_.join();  // 必须等上一个线程结束
    }

    for (auto dev : GetDevParam().uartVoice) {
        uart_ = std::make_unique<Uart>(dev);

        if (uart_->IsOpen() == true) {
            uart_->SetParam(9600, 8, 'N', 1);
            uart_->SetBlockCnt(4);

            isOk_ = true;
            thread_ = std::thread(&voiceNode::Loop, this);  // 析构由基类负责
            break;                                          // 成功打开则不再查询下一个设备号
        }
    }
}

/**
 * @brief
 * @description: 串口接收处理
 * @return {}
 *
 */
void voiceNode::Loop()
{
    while (isOk_) {
        // 该变量用于判断是否接收到了一个正确的包，而不是一组（完整包含0x51 0x52 0x53）的包
        //  读取串口数据并存入队列
        auto data = uart_->Read(1);  // 超时1ms
        for (const auto& var : data) {
            uartData_.push(var);
        }
        // 解析
        auto [VoiceDataValid, VoiceData] = ParseData(uartData_);
        if (VoiceDataValid == true) {
            // 发送
            // MsgSend send("bs::OfflineVoice");
            // send.Send(VoiceData);
            MsgTrySend("bs::OfflineVoice", VoiceData);
        }
    }
}

std::pair<bool, voiceNode::voiceMessage> voiceNode::ParseData(std::queue<uint8_t>& q)
{
    bool dataValid = false;
    voiceNode::voiceMessage VoiceData = voiceMessage::CmdNull;  // update默认为false
    // 当队列数据少于4个时，即不满一帧了
    while (q.size() >= 4) {
        // 寻找协议头，只有找到就取出4个，理论上只有在程序刚运行时丢失数据
        if (q.front() == 0x68) {
            uint8_t buf[4] = {0};
            for (int i = 0; i < 4; ++i) {
                buf[i] = q.front();
                q.pop();
            }

            if (0x16 == buf[3]) {
                if (buf[1] == buf[2]) {
                    dataValid = true;
                    switch (buf[1]) {
                        case 1:
                            VoiceData = voiceMessage::CmdNameCall;
                            LOG_DEBUG("TxCmdNameCall");

                            break;
                        case 2:
                            VoiceData = voiceMessage::CmdStandUp;
                            LOG_DEBUG("TxCmdStandUp");

                            break;
                        case 3:
                            VoiceData = voiceMessage::CmdLieDown;
                            LOG_DEBUG("TxCmdLieDown");

                            break;
                        case 4:
                            VoiceData = voiceMessage::CmdMarchInPlace;
                            LOG_DEBUG("TxCmdMarchInPlace");

                            break;
                        case 5:
                            VoiceData = voiceMessage::CmdDance;
                            LOG_DEBUG("TxCmdDance");
                            break;
                        case 6:
                            VoiceData = voiceMessage::CmdTurnRight;
                            LOG_DEBUG("TxCmdTurn");
                            break;
                        case 7:
                            VoiceData = voiceMessage::CmdTurnLeft;
                            LOG_DEBUG("TxCmdTurnLeft");
                            break;
                        case 8:
                            VoiceData = voiceMessage::CmdMoveBackward;
                            LOG_DEBUG("TxCmdMoveBackward");
                            break;
                        case 9:
                            VoiceData = voiceMessage::CmdMoveForward;
                            LOG_DEBUG("TxCmdMoveForward");
                            break;
                        case 10:
                            VoiceData = voiceMessage::CmdMoveLeft;
                            LOG_DEBUG("TxCmdMoveLeft");
                            break;
                        case 11:
                            VoiceData = voiceMessage::CmdMoveRight;
                            LOG_DEBUG("TxCmdMoveRight");
                            break;
                        case 12:
                            VoiceData = voiceMessage::CmdStandTaller;
                            LOG_DEBUG("TxCmdStandTaller");
                            break;
                        case 13:
                            VoiceData = voiceMessage::CmdCrouchLower;
                            LOG_DEBUG("TxCmdCrouchLower");
                            break;
                        case 14:
                            VoiceData = voiceMessage::CmdStop;
                            LOG_DEBUG("TxCmdStop");
                            break;
                        case 15:
                            VoiceData = voiceMessage::Shutdown;
                            LOG_DEBUG("TxShutdown");
                            break;
                        default:
                            LOG_DEBUG("TxNull");
                            dataValid = false;
                            break;
                    }
                }
            }
        } else {
            q.pop();
        }
    }
    return {dataValid, VoiceData};
}

void voiceNode::ErrorDetect() {}
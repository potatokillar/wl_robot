/*
默认夹爪最高温度为75摄氏度，该最高温度可以被设置，但本API预计不支持设置最温度
*/

#pragma once
#include <mutex>
#include <queue>

#include "baseline.hpp"
#include "canProtocol.hpp"
#include "deviceParam.hpp"
#include "uart.hpp"
template <typename... Args>
inline void LOG_TRACE_GRIPPER(const std::string &fmt, Args &&...args)
{
    GetLog().Trace("gripper", fmt, std::forward<Args>(args)...);
}

// 调试，调试类信息。面向开发使用
template <typename... Args>
inline void LOG_DEBUG_GRIPPER(const std::string &fmt, Args &&...args)
{
    GetLog().Debug("gripper", fmt, std::forward<Args>(args)...);
}

// 信息，一般提示类信息
template <typename... Args>
inline void LOG_INFO_GRIPPER(const std::string &fmt, Args &&...args)
{
    GetLog().Info("gripper", fmt, std::forward<Args>(args)...);
}

// 警告，程序可以运行且无错，但需要额外注意，例如算法计算超限，程序运行超长，手柄拔出等
template <typename... Args>
inline void LOG_WARN_GRIPPER(const std::string &fmt, Args &&...args)
{
    GetLog().Warn("gripper", fmt, std::forward<Args>(args)...);
}

// 错误，该信息表示程序可以运行，但结果不可信。例如IMU或SPI未正常启动等
template <typename... Args>
inline void LOG_ERROR_GRIPPER(const std::string &fmt, Args &&...args)
{
    GetLog().Error("gripper", fmt, std::forward<Args>(args)...);
}

// 致命，该信息表示程序将无法运行，例如相关类未起来
template <typename... Args>
inline void LOG_CRITICAL_GRIPPER(const std::string &fmt, Args &&...args)
{
    GetLog().Critical("gripper", fmt, std::forward<Args>(args)...);
}

static constexpr u8 GRIPPER_UART_LEN = 10;                          // 标识符2+有效数据8
static constexpr u8 GRIPPER_UART_TAG = 0xAA;                        // 0xAA表示是can帧的tag
static constexpr u8 GRIPPER_UART_TOTAL = GRIPPER_UART_LEN + 2 + 1;  // 头1+len1+标识符2+有效数据8+1校验位

struct GripperData
{
    u8 boardId;     // 板子ID，0~16，根据硬件不同而不同 ， todo发送的时候应该是00，实现上通信板不检查这个字段
    u8 version;     // 0预留，实现上通信板不检查这个字段
    u8 cmd;         // 功能码，区分功能
    u8 payload[7];  // 剩下7个有效数据域
    u8 add;         // 累加和
                    // 目前只支持标准帧
    GripperData(const u8 data[])
    {
        version = (data[0] >> 5) & 0x03;  // 5~6bit，现在改为预留，无功能
        boardId = data[1] & 0x0F;         // 8~11bit=0~4bit，地址
        cmd = data[2];                    // 功能码
        ::memcpy(payload, &data[3], 7);
        add = boardId + version + cmd;
        for (int i = 0; i < 7; i++) {
            add += payload[i];
        }
    }
    // 下面这个构造函数用于需要返回消息的命令，现在好像只有急停需要调用这个
    GripperData(u8 _cmd)
    {
        boardId = 0;
        version = 0;
        cmd = _cmd;
        ::memset(payload, 0, 7);  // 清空数据域的后四个字节，
    }

    std::vector<u8> GetVector() const
    {
        std::vector<u8> data;
        data.push_back(GRIPPER_UART_TAG);
        data.push_back(GRIPPER_UART_LEN);
        data.push_back(version << 5);
        data.push_back(boardId);
        data.push_back(cmd);
        for (auto i = 0; i < 7; i++) {
            data.push_back(payload[i]);
        }
        data.push_back(add);

        return data;
    }
};

// using CanRxCallType = std::function<void(const GripperData &can)>;

// 夹爪用到的串口类
class GripperCanProtocol
{
public:
    // 默认构造函数
    GripperCanProtocol()
    {
        if (GetDevArmParam().uartGripper.empty() == false) {
            if (GetSerialPort().Open(UART_NAME, GetDevArmParam().uartGripper[0]) == false) {
                LOG_TRACE_GRIPPER("open error");
                return;
            }
            if (GetSerialPort().SetParam(UART_NAME, 115200) == false) {
                LOG_TRACE_GRIPPER("set param error");
                return;
            }
            // 夹爪不接受串口数据
            //  GetSerialPort().AsyncRead(UART_NAME, [this](const std::vector<u8> &data) { this->RecvDeal(data); });

            GetSerialPort().SetWriteCompleteCallback(UART_NAME, [this](const boost::system::error_code &e, std::size_t t) { this->SendComplete(e, t); });

        } else {
            LOG_TRACE_GRIPPER("no serial port");
        }
    }

    // 虚析构函数
    ~GripperCanProtocol() {}

    // 添加接受数据的回调函数的对外接口
    // bool AddRxCall(u8 cmd, CanRxCallType func);

    // 发送的接口
    void Send(const GripperData &data)
    {
        // std::lock_guard<std::mutex> lock(txMutex_);
        // txFifo_.push(data.GetVector());

        if (noMsgSend == true) {
            noMsgSend = false;
            GetSerialPort().AsyncWrite(UART_NAME, data.GetVector());
        }
    }

private:
    u8 oneMsg[GRIPPER_UART_LEN];

    // 接受回调函数，本应用没有串口接收
    // void RecvDeal(const std::vector<u8> &data);

    // 接收到的数据，本应用没有串口接收
    // std::queue<u8> rxFifo_;

    // 接受数据后用于调用回调函数的表
    // std::map<u8, CanRxCallType> funcMap_;

    // 发送完成回调函数，本应用没有串口接收
    void SendComplete(const boost::system::error_code &, std::size_t)
    {
        // std::lock_guard<std::mutex> lock(txMutex_);  // 该函数由异步调用，和Send不是一个线程
        // txFifo_.pop();
        // if (txFifo_.empty() == false) {
        //     GetSerialPort().AsyncWrite(UART_NAME, txFifo_.front());
        // } else {
        //     noMsgSend = true;
        // }
        noMsgSend = true;
    }

    // std::queue<std::vector<u8>> txFifo_;  // 待发送的数据
    // std::mutex txMutex_;                  // 用于txFifo_变量的锁

    bool noMsgSend{true};  // 当前无数据发送
    const std::string UART_NAME{"uartGripper"};
};

/**这里是一个跨线程通信的API，用于给其他线程通过串口控制夹爪*/
class ControlGripperAPI
{
public:
    // 夹爪收到移动命令是，如果处于夹爪断电状态，则会自动打开电源
    void PositionPercentageNew(uint8_t percentage, double speed);
    void GetGripperPowerOff();

    void AotuGetGripperOff(double speed);
    void AotuGetGripperOn(double speed);
    bool Gripper01New(u8 percentage, double speed);

private:
    GripperCanProtocol canProtocol_;
};

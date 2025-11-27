
#pragma once
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <queue>
#include <thread>

#include "baseline.hpp"
#include "serialPort.hpp"
#include "uart.hpp"

static constexpr u8 UART_LEN = 10;
struct SensorCanData
{
    u8 boardId;
    u8 version;
    u8 cmd;
    u8 payload[7];
    u8 add;
    SensorCanData(const u8 data[])
    {
        version = (data[0] >> 5) & 0x03;
        boardId = data[1] & 0x0F;
        cmd = data[2];
        ::memcpy(payload, &data[3], 7);
    }

    SensorCanData(u8 _cmd)
    {
        boardId = 0;
        version = 0;
        cmd = _cmd;
    }

    std::vector<u8> GetVector() const;
};

using CanRxCallType = std::function<void(const SensorCanData &can)>;

class CanProtocol
{
public:
    CanProtocol();
    virtual ~CanProtocol();

    bool AddRxCall(u8 cmd, CanRxCallType func);
    void Send(const SensorCanData &data);

private:
    u8 oneMsg[UART_LEN];
    void RecvDeal(const std::vector<u8> &data);
    void SendComplete(const boost::system::error_code &, std::size_t);
    std::queue<u8> rxFifo_;               // 接收到的数据
    std::queue<std::vector<u8>> txFifo_;  // 待发送的数据
    std::mutex txMutex_;
    std::map<u8, CanRxCallType> funcMap_;
    bool noMsgSend{true};
    const std::string UART_NAME{"canBridge"};
};

template <typename... Args>
inline void LOG_TRACE_SENSOR(const std::string &fmt, Args &&...args)
{
    GetLog().Trace("sensor", fmt, std::forward<Args>(args)...);
}

// 调试，调试类信息。面向开发使用
template <typename... Args>
inline void LOG_DEBUG_SENSOR(const std::string &fmt, Args &&...args)
{
    GetLog().Debug("sensor", fmt, std::forward<Args>(args)...);
}

// 信息，一般提示类信息
template <typename... Args>
inline void LOG_INFO_SENSOR(const std::string &fmt, Args &&...args)
{
    GetLog().Info("sensor", fmt, std::forward<Args>(args)...);
}

// 警告，程序可以运行且无错，但需要额外注意，例如算法计算超限，程序运行超长，手柄拔出等
template <typename... Args>
inline void LOG_WARN_SENSOR(const std::string &fmt, Args &&...args)
{
    GetLog().Warn("sensor", fmt, std::forward<Args>(args)...);
}

// 错误，该信息表示程序可以运行，但结果不可信。例如IMU或SPI未正常启动等
template <typename... Args>
inline void LOG_ERROR_SENSOR(const std::string &fmt, Args &&...args)
{
    GetLog().Error("sensor", fmt, std::forward<Args>(args)...);
}

// 致命，该信息表示程序将无法运行，例如相关类未起来
template <typename... Args>
inline void LOG_CRITICAL_SENSOR(const std::string &fmt, Args &&...args)
{
    GetLog().Critical("sensor", fmt, std::forward<Args>(args)...);
}

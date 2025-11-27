
#pragma once
#include <netinet/in.h>

#include <string>

#include "baseline.hpp"

template <typename... Args>
inline void LOG_TRACE_SDK(const std::string &fmt, Args &&...args)
{
    GetLog().Trace("sdk", fmt, std::forward<Args>(args)...);
}

// 调试，调试类信息。面向开发使用
template <typename... Args>
inline void LOG_DEBUG_SDK(const std::string &fmt, Args &&...args)
{
    GetLog().Debug("sdk", fmt, std::forward<Args>(args)...);
}

// 信息，一般提示类信息
template <typename... Args>
inline void LOG_INFO_SDK(const std::string &fmt, Args &&...args)
{
    GetLog().Info("sdk", fmt, std::forward<Args>(args)...);
}

// 警告，程序可以运行且无错，但需要额外注意，例如算法计算超限，程序运行超长，手柄拔出等
template <typename... Args>
inline void LOG_WARN_SDK(const std::string &fmt, Args &&...args)
{
    GetLog().Warn("sdk", fmt, std::forward<Args>(args)...);
}

// 错误，该信息表示程序可以运行，但结果不可信。例如IMU或SPI未正常启动等
template <typename... Args>
inline void LOG_ERROR_SDK(const std::string &fmt, Args &&...args)
{
    GetLog().Error("sdk", fmt, std::forward<Args>(args)...);
}

// 致命，该信息表示程序将无法运行，例如相关类未起来
template <typename... Args>
inline void LOG_CRITICAL_SDK(const std::string &fmt, Args &&...args)
{
    GetLog().Critical("sdk", fmt, std::forward<Args>(args)...);
}

struct NetClientInfo
{
    std::string serverName;
    std::string clientIp;
    uint16_t clientPort;
    uint32_t lost;

    bool operator==(const NetClientInfo &other) const { return (clientIp == other.clientIp && clientPort == other.clientPort /* && other member comparisons if any */); }

    bool operator<(const NetClientInfo &other) const
    {
        return (clientIp < other.clientIp || (clientIp == other.clientIp && clientPort < other.clientPort) /* && other member comparisons if any */);
    }
};

std::pair<bool, struct sockaddr_in> String2Sockaddr(const std::string &ip, uint16_t port);
std::tuple<bool, std::string, uint16_t> Sockaddr2String(const struct sockaddr_in &addr);

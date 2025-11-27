
#pragma once
#include "baseline.hpp"

// arm机械臂型号
enum class ArmModel
{
    unused,  // 不启用
    base,    //
    arm,
    iris,
    qa01,
    ga701,
    joint,  // 关节级控制
};

enum class NetMode
{
    base,
    can,
    ethercat,
};

template <typename... Args>
inline void LOG_TRACE_ARM(const std::string &fmt, Args &&...args)
{
    GetLog().Trace("arm", fmt, std::forward<Args>(args)...);
}

// 调试，调试类信息。面向开发使用
template <typename... Args>
inline void LOG_DEBUG_ARM(const std::string &fmt, Args &&...args)
{
    GetLog().Debug("arm", fmt, std::forward<Args>(args)...);
}

// 信息，一般提示类信息
template <typename... Args>
inline void LOG_INFO_ARM(const std::string &fmt, Args &&...args)
{
    GetLog().Info("arm", fmt, std::forward<Args>(args)...);
}

// 警告，程序可以运行且无错，但需要额外注意，例如算法计算超限，程序运行超长，手柄拔出等
template <typename... Args>
inline void LOG_WARN_ARM(const std::string &fmt, Args &&...args)
{
    GetLog().Warn("arm", fmt, std::forward<Args>(args)...);
}

// 错误，该信息表示程序可以运行，但结果不可信。例如IMU或SPI未正常启动等
template <typename... Args>
inline void LOG_ERROR_ARM(const std::string &fmt, Args &&...args)
{
    GetLog().Error("arm", fmt, std::forward<Args>(args)...);
}

// 致命，该信息表示程序将无法运行，例如相关类未起来
template <typename... Args>
inline void LOG_CRITICAL_ARM(const std::string &fmt, Args &&...args)
{
    GetLog().Critical("arm", fmt, std::forward<Args>(args)...);
}

namespace arm
{

}  // namespace arm
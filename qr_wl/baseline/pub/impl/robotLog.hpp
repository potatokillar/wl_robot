
#pragma once
#include <cstdio>
#include <iostream>
#include <map>
#include <string>

#include "spdlog/sinks/basic_file_sink.h"
#include "spdlog/sinks/rotating_file_sink.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/spdlog.h"

// LOG输出位置
enum class LogPosi
{
    console,  // 只输出到终端
    file,     // 只输出到文件
    all,      // 均输出
};

// 每一个模块的日志，都支持设置不同的文件，等级，位置
// 但当前未开放不同文件写入不同的日志文件
struct RobotLogImpl
{
    std::string name;
    std::shared_ptr<spdlog::sinks::basic_file_sink_mt> fileSink;  // 指向文件的sink
    std::shared_ptr<spdlog::logger> log;                          // 日志实例
    LogPosi pos;                                                  // 日志输出位置
    spdlog::level::level_enum level;                              // 日志输出级别
};

class RobotLog
{
public:
    static RobotLog &GetInstance()
    {
        static RobotLog instance;
        return instance;
    }
    void SetLevel(int level);

    template <typename... Args>
    inline void Trace(const std::string &name, const std::string &fmt, Args &&...args)
    {
        Log(name)->trace("[" + name + "] " + fmt, std::forward<Args>(args)...);
    }

    template <typename... Args>
    inline void Debug(const std::string &name, const std::string &fmt, Args &&...args)
    {
        Log(name)->debug("[" + name + "] " + fmt, std::forward<Args>(args)...);
    }

    template <typename... Args>
    inline void Info(const std::string &name, const std::string &fmt, Args &&...args)
    {
        Log(name)->info("[" + name + "] " + fmt, std::forward<Args>(args)...);
    }

    template <typename... Args>
    inline void Warn(const std::string &name, const std::string &fmt, Args &&...args)
    {
        Log(name)->warn("[" + name + "] " + fmt, std::forward<Args>(args)...);
    }

    template <typename... Args>
    inline void Error(const std::string &name, const std::string &fmt, Args &&...args)
    {
        Log(name)->error("[" + name + "] " + fmt, std::forward<Args>(args)...);
    }

    template <typename... Args>
    inline void Critical(const std::string &name, const std::string &fmt, Args &&...args)
    {
        Log(name)->critical("[" + name + "] " + fmt, std::forward<Args>(args)...);
    }

private:
    RobotLog();
    void AddModule(const std::string &name);
    void ParseToml(RobotLogImpl *in);
    std::shared_ptr<spdlog::logger> Log(const std::string &name);

    std::shared_ptr<spdlog::sinks::basic_file_sink_mt> fileSink_;       // 指向文件的sink
    std::shared_ptr<spdlog::sinks::stdout_color_sink_mt> consoleSink_;  // 指向终端的sink
    std::map<std::string, RobotLogImpl> logs_;
};
inline RobotLog &GetLog() { return RobotLog::GetInstance(); }

/******************************************************************
 * 日志级别 trace < debug < info < warn < err < critical 依次递增
 * 当日志输出设置为某一级别时，大于等于该级别的将被输出，小于的不输出
 * 例如设置为info，则输出info/warn/err/critical，不输出trace/debug
 * 可选择三种输出方式，仅输出到终端，仅输出到文件，两者同时输出
 *
 * ****************************************************************/
// 封装API接口
// 最低级别，可用于临时的频繁打印信息
template <typename... Args>
inline void LOG_TRACE(const std::string &fmt, Args &&...args)
{
    GetLog().Trace("robot", fmt, std::forward<Args>(args)...);
}

// 调试，调试类信息。面向开发使用
template <typename... Args>
inline void LOG_DEBUG(const std::string &fmt, Args &&...args)
{
    GetLog().Debug("robot", fmt, std::forward<Args>(args)...);
}

// 信息，一般提示类信息
template <typename... Args>
inline void LOG_INFO(const std::string &fmt, Args &&...args)
{
    GetLog().Info("robot", fmt, std::forward<Args>(args)...);
}

// 警告，程序可以运行且无错，但需要额外注意，例如算法计算超限，程序运行超长，手柄拔出等
template <typename... Args>
inline void LOG_WARN(const std::string &fmt, Args &&...args)
{
    GetLog().Warn("robot", fmt, std::forward<Args>(args)...);
}

// 错误，该信息表示程序可以运行，但结果不可信。例如IMU或SPI未正常启动等
template <typename... Args>
inline void LOG_ERROR(const std::string &fmt, Args &&...args)
{
    GetLog().Error("robot", fmt, std::forward<Args>(args)...);
}

// 致命，该信息表示程序将无法运行，例如相关类未起来
template <typename... Args>
inline void LOG_CRITICAL(const std::string &fmt, Args &&...args)
{
    GetLog().Critical("robot", fmt, std::forward<Args>(args)...);
}

// ANSI 转义码
const std::string LOG_RESET{"\033[0m"};
const std::string LOG_RED{"\033[31m"};
const std::string LOG_GREEN{"\033[32m"};
const std::string LOG_YELLOW{"\033[33m"};
const std::string LOG_BLUE{"\033[34m"};
const std::string LOG_MAGENTA{"\033[35m"};
const std::string LOG_CYAN{"\033[36m"};
const std::string LOG_WHITE{"\033[37m"};
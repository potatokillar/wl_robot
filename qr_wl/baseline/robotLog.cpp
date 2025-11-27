
#include <chrono>
#include <filesystem>
#include <map>

#include "baseline.hpp"
#include "spdlog/common.h"
#include "spdlog/sinks/ansicolor_sink.h"

using namespace std;
namespace fs = std::filesystem;

RobotLog::RobotLog()
{
    auto t = chrono::system_clock::to_time_t(chrono::system_clock::now());
    stringstream ss;

    ss << std::put_time(std::localtime(&t), "%F~%H.%M.%S");
    fs::path logname = fs::absolute("log/qrLog" + ss.str() + ".txt");

    fileSink_ = std::make_shared<spdlog::sinks::basic_file_sink_mt>(logname.c_str(), true);
    consoleSink_ = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
}

void RobotLog::AddModule(const string& name)
{
    if (logs_.count(name)) {
        return;
    }

    RobotLogImpl impl;

    impl.fileSink = fileSink_;
    impl.name = name;

    ParseToml(&impl);

    shared_ptr<spdlog::logger> robotLog;
    if (impl.pos == LogPosi::all) {
        spdlog::sinks_init_list sink_lst{consoleSink_, impl.fileSink};
        robotLog = make_shared<spdlog::logger>(name, sink_lst.begin(), sink_lst.end());
    } else if (impl.pos == LogPosi::file) {
        robotLog = make_shared<spdlog::logger>(name, impl.fileSink);
    } else {
        robotLog = make_shared<spdlog::logger>(name, consoleSink_);
    }

    robotLog->set_pattern("[%H:%M:%S-%e][%^%l]%v%$");
    robotLog->set_level(impl.level);
    robotLog->flush_on(impl.level);
    spdlog::register_logger(robotLog);

    impl.log = robotLog;
    logs_[name] = impl;
}

shared_ptr<spdlog::logger> RobotLog::Log(const string& name)
{
    if (logs_.count(name) == 0) {
        AddModule(name);
    }
    return logs_.at(name).log;
}

void RobotLog::ParseToml(RobotLogImpl* in)
{
    // 输出目标设置
    auto saveLog = GetRobotConfigDirect<bool>("saveLog");
    if (saveLog) {
        if (saveLog.value() == true) {
            in->pos = LogPosi::all;
        } else {
            in->pos = LogPosi::console;
        }
    }

    auto logLevel = GetRobotConfigDirect<int>("logLevel");
    if (logLevel) {
        if ((logLevel.value() >= 0) && (logLevel.value() <= 6)) {
            in->level = (spdlog::level::level_enum)logLevel.value();
        }
    }

    auto subLogLevel = GetRobotConfigDirect<int>("subLogLevel", in->name.c_str());
    if (subLogLevel) {
        if ((subLogLevel.value() >= 0) && (subLogLevel.value() <= 6)) {
            in->level = (spdlog::level::level_enum)subLogLevel.value();
        }
    }
}

void RobotLog::SetLevel(int level)
{
    if ((level >= 0) && (level <= 6)) {
        for (auto& v : logs_) {
            v.second.log->set_level((spdlog::level::level_enum)level);
        }
    }
}
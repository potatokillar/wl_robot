
#pragma once

#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>

#include "PeriodicTask.h"
#include "robotType.hpp"
// #include "robotType.hpp"

using SoftTimerFunc = std::function<void(void)>;

struct SoftTimerDeal
{
    u64 now{0};     // 当前计数时间
    u64 period{0};  // 定时器间隔，毫秒
    SoftTimerFunc func;
    bool isHold{true};  // 该定时器是否保持，若为false，则在运行完毕后删除
};

// 每一个实例都会生成一个线程
class SoftTimer
{
public:
    SoftTimer();
    virtual ~SoftTimer();
    bool AddFunc(const std::string& name, u32 ms, SoftTimerFunc func, bool isHold = true);
    bool RmvFunc(const std::string& name);

private:
    void Loop();
    std::map<std::string, SoftTimerDeal> deal_;
    std::unique_ptr<PeriodicMemberFunction<SoftTimer>> task_;
    std::mutex mutex_;
};

extern std::map<std::size_t, SoftTimer> SoftTimerSet;
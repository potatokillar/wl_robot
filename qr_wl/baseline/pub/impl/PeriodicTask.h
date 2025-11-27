
#pragma once

#include <chrono>
#include <deque>
#include <string>
#include <thread>
#include <vector>

#include "timerTools.hpp"

/*!
 * A single periodic task which will call run() at the given frequency
 */
class PeriodicTask
{
public:
    PeriodicTask(const std::string& name, double period, bool rtTask = false);
    virtual ~PeriodicTask() { Stop(); }

    void Start();
    void Stop();
    virtual void Init() = 0;     // 启动线程前的初始化操作
    virtual void Run() = 0;      // 具体的线程运行函数
    virtual void Cleanup() = 0;  // 线程停止后的清理操作
    /*!
     * 该任务的定义周期
     */
    double GetPeriod() const { return period_; }

private:
    void PrintStatisticsInfo(uint64_t runTime, uint64_t lastPeriod);

private:
    void LoopFunction();

    volatile bool running_ = false;
    uint64_t lastStartTime_ = 0;  // 上一次线程启动的时间点
    uint64_t period_ = 0;         // 定义周期时间，单位均微妙
    std::string name_;            // 构造函数的初始化顺序应该同声明顺序一致
    bool rtTask_;
    std::thread thread_;
    uint64_t lastPrint_ = 0;
    std::deque<uint64_t> runTimeSet_;  // 运行时间的集合
};

/*!
 * 用于周期调用非成员函数的周期任务类
 */
class PeriodicFunction : public PeriodicTask
{
public:
    PeriodicFunction(const std::string& name, double period, void (*function)(), bool rtTask = false) : PeriodicTask(name, period, rtTask), function_(function) {}

    void Init() override {}
    void Run() override { function_(); }
    void Cleanup() override {}

private:
    void (*function_)() = nullptr;
};

/*!
 * 用于调用成员函数的周期任务
 */
template <typename T>
class PeriodicMemberFunction : public PeriodicTask
{
public:
    PeriodicMemberFunction(const std::string& name, double period, T* obj, void (T::*function)(), bool rtTask = false)
        : PeriodicTask(name, period, rtTask), function_(function), obj_(obj)
    {
    }

    void Init() override {}
    void Run() override { (obj_->*function_)(); }
    void Cleanup() override {}

private:
    void (T::*function_)() = nullptr;
    T* obj_;
};

#include <sched.h>
#include <sys/sysinfo.h>

#include "baseline.hpp"

using namespace std;

// CPU亲和性，序号是CPU核序号，内容是任务名
static std::vector<std::string> cpuAffinty;

/****************************************************************
 * 功能：构造函数
 * 输入：@1 任务名，用于提示
 *      @2  任务周期，单位为秒
 *      @3  任务管理器，将该任务纳入任务管理器范畴
 *      @4  是否为实时任务，默认下实时任务纳入管理
 * 输出：@1
 *      @2
 ****************************************************************/
PeriodicTask::PeriodicTask(const std::string& name, double period, bool rtTask) : name_(name), rtTask_(rtTask)
{
    period_ = period * 1000000;  // 转成微秒
    if (cpuAffinty.empty()) {
        cpuAffinty.resize(get_nprocs());
        LOG_DEBUG("available nprocs:{}", get_nprocs());
    }
}

/**
 * @description: 启动线程
 * @return {*}
 */
void PeriodicTask::Start()
{
    if (running_) {
        LOG_WARN("[PeriodicTask] Tried to start {} but it was already running!", name_.c_str());
        return;
    }

    Init();
    running_ = true;
    thread_ = thread(&PeriodicTask::LoopFunction, this);

    if (rtTask_) {
        // 设置线程优先级
        sched_param sch;
        int policy;
        pthread_getschedparam(thread_.native_handle(), &policy, &sch);
        sch.sched_priority = 99;
        if (pthread_setschedparam(thread_.native_handle(), SCHED_FIFO, &sch)) {
            LOG_ERROR("{} failed to setschedparam, you maybe exec as root", name_);
        }

        // 分配CPU亲和性，第0个CPU固定不分配
        int cpu = 0;
        cpu_set_t mask;
        CPU_ZERO(&mask);
        for (size_t i = 1; i < cpuAffinty.size(); i++) {
            if ((cpuAffinty[i] == name_) || (cpuAffinty[i].empty() == true)) {
                cpu = i;
                cpuAffinty[i] = name_;

                LOG_DEBUG("periodicTask {} be assigned to cpu {}", name_, cpu);
                CPU_SET(cpu, &mask);
                if (pthread_setaffinity_np(thread_.native_handle(), sizeof(cpu_set_t), &mask) != 0) {
                    LOG_WARN("{} cpu setaffinity error", name_);
                }

                break;
            }
        }
    }

    // 防止误报
    lastStartTime_ = TimerTools::GetNowTickUs();
}

/**
 * @description: 停止任务
 * @return {*}
 */
void PeriodicTask::Stop()
{
    if (!running_) {
        // LOG_WARN("[PeriodicTask] Tried to stop {} but it wasn't running!", name_.c_str());
        return;
    }
    running_ = false;
    // LOG_INFO("waiting for {} to stop...", name_);
    if (thread_.joinable()) {
        thread_.join();
    }

// 删除已存储的亲和性
#if 0
    if (rtTask_ == true) {
        for (auto& v : cpuAffinty) {
            if (v == name_) {
                v.clear();
            }
        }
    }
#endif
    LOG_INFO("periodicTask:{} stop done!", name_);
}

/****************************************************************
 * 功能：周期函数，使用C++11重写MIT的
 * 输入：@1
 * 输出：@1
 *      @2
 ****************************************************************/
void PeriodicTask::LoopFunction()
{
    while (running_) {
        auto start = TimerTools::GetNowTickUs();  // 记录任务运行前的时间
        Run();                                    // 运行线程
        auto end = TimerTools::GetNowTickUs();    // 记录任务运行后的时间

        if (rtTask_ == true) {
            PrintStatisticsInfo(end - start, start - lastStartTime_);
        }

        // 本次与上一次的周期间隔
        lastStartTime_ = start;  // 记录本次启动的时间

        TimerTools::SleepUntilUs(start + period_);
    }
    // LOG_WARN("[PeriodicTask] {} has stopped!", name_);
}

/**
 * @description: 打印统计信息
 * @param runTime 本次运行时间
 * @param lastPeriod 上次启动到这次启动的调度周期
 * @return {}
 */
void PeriodicTask::PrintStatisticsInfo(uint64_t runTime, uint64_t lastPeriod)
{
    if (runTimeSet_.empty() == false) {
        auto maxT = std::max(runTimeSet_.back(), period_);
        if (lastPeriod > maxT * 1.3) {
            //  LOG_WARN("sched warnning, name:{}, runTime:{}, lastPeriod:{}", name_, runTime, lastPeriod);
        }
    }

    if (runTime > period_ * 1.5) {
        // LOG_WARN("thread run too long, {}, {}", name_, runTime);
    }

    runTimeSet_.push_back(runTime);  // 保存本次运行时间

    if (TimerTools::GetNowTickMs() - lastPrint_ > 10000) {
        if (lastPrint_ == 0) {
            runTimeSet_.clear();  // 刚启动的时候，会有一个忙延时，去除这个误报警
            lastPrint_ = TimerTools::GetNowTickMs();
            return;
        }
        uint64_t runTimeSum = 0;
        uint32_t lessCnt = 0;
        uint32_t moreCnt = 0;
        uint64_t max = 0;
        for (const auto& var : runTimeSet_) {
            runTimeSum += var;
            // 运行时间小于定义周期60%的个数
            if (var < period_ * 0.6) {
                lessCnt++;
            }
            if (var > period_ * 1.3) {
                moreCnt++;
            }
            max = std::max(max, var);
        }
        auto total = runTimeSet_.size();
        if (moreCnt > 100) {  // 行能不足
            LOG_WARN("|name    |period  |count   |avgRun  |maxRun  |run<60% |run>130%|\n{:24}|{:<8}|{:<8}|{:<8}|{:<8}|{:<8}|{:<8}|{:<8}|",
                     ' ',
                     name_,
                     period_,
                     total,
                     runTimeSum / total,
                     max,
                     lessCnt,
                     moreCnt);
        }
        runTimeSet_.clear();
        lastPrint_ = TimerTools::GetNowTickMs();
    }
}
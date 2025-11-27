
#pragma once

#include <chrono>
#include <thread>

// 记录开机启动的时间
static std::chrono::steady_clock::time_point bootTime = std::chrono::steady_clock::now();
class TimerTools
{
public:
    /**
     * @description: 获取程序启动后到现在的时间
     * @param {*}
     * @return {*}
     */
    static uint64_t GetNowTickUs()
    {
        auto nowTick = std::chrono::steady_clock::now();
        return std::chrono::duration_cast<std::chrono::microseconds>(nowTick - bootTime).count();
    }
    static uint64_t GetNowTickMs()
    {
        auto nowTick = std::chrono::steady_clock::now();
        return std::chrono::duration_cast<std::chrono::milliseconds>(nowTick - bootTime).count();
    }
    static uint64_t GetNowTickS()
    {
        auto nowTick = std::chrono::steady_clock::now();
        return std::chrono::duration_cast<std::chrono::seconds>(nowTick - bootTime).count();
    }

    /**
     * @description: 睡眠一段时间，从当前时间开始后计算
     * @param {uint32_t} 睡眠的持续时间
     * @return {*}
     */
    static void SleepForUs(uint32_t us) { std::this_thread::sleep_for(std::chrono::microseconds(us)); }
    static void SleepForMs(uint32_t ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }
    static void SleepForS(uint32_t s) { std::this_thread::sleep_for(std::chrono::seconds(s)); }

    /**
     * @description: 睡眠到一个时间点后
     * @param {uint64_t} 醒来的时间点
     * @return {*}
     */
    static void SleepUntilUs(uint64_t nextUs) { std::this_thread::sleep_until(bootTime + std::chrono::microseconds(nextUs)); }
    static void SleepUntilMs(uint64_t nextMs) { std::this_thread::sleep_until(bootTime + std::chrono::milliseconds(nextMs)); }
    static void SleepUntilS(uint64_t nextUs) { std::this_thread::sleep_until(bootTime + std::chrono::seconds(nextUs)); }

    static std::chrono::microseconds ToUs(uint32_t us) { return std::chrono::microseconds(us); }
    static std::chrono::milliseconds ToMs(uint32_t ms) { return std::chrono::milliseconds(ms); }
    static std::chrono::seconds ToS(uint32_t s) { return std::chrono::seconds(s); }

private:
};

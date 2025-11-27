
#include "baseline.hpp"

using namespace ::std;

static constexpr int PER_DELAY = 10;

std::map<std::size_t, SoftTimer> SoftTimerSet;

SoftTimer::SoftTimer()
{
    static u32 count = 0;  // 多少个定时器，用于命名
    string name = "sfTimer" + to_string(count);
    task_ = make_unique<PeriodicMemberFunction<SoftTimer>>(name, PER_DELAY * 0.001, this, &SoftTimer::Loop);
    task_->Start();
    count++;
}

SoftTimer::~SoftTimer() { task_->Stop(); }

bool SoftTimer::AddFunc(const std::string& name, u32 ms, SoftTimerFunc func, bool isHold)
{
    lock_guard<mutex> lock(mutex_);
    if (deal_.count(name) > 0) {
        LOG_ERROR("softTimer has a same name!");
        return false;
    } else {
        //  LOG_INFO("softTimer insert success {}", name);
    }

    deal_[name].period = ms / PER_DELAY;
    deal_[name].now = ms / PER_DELAY;
    deal_[name].func = func;
    deal_[name].isHold = isHold;
    LOG_DEBUG("SoftTimer Add, name: {}, ms: {} ", name, ms);
    return true;
}

/**
 * @description: 移除一个回调函数
 * @param name
 * @return {}
 */
bool SoftTimer::RmvFunc(const std::string& name)
{
    lock_guard<mutex> lock(mutex_);
    auto search = deal_.find(name);
    if (search != deal_.end()) {
        deal_.erase(search);
    } else {
        LOG_ERROR("softTimer has no the name!");
        return false;
    }
    return true;
}

void SoftTimer::Loop()
{
    lock_guard<mutex> lock(mutex_);

    for (auto iter = deal_.begin(); iter != deal_.end();) {
        iter->second.now--;

        if (iter->second.now == 0) {
            iter->second.func();
            iter->second.now = iter->second.period;

            if (iter->second.isHold == false) {
                iter = deal_.erase(iter);
                continue;
            }
        }
        ++iter;
    }
}
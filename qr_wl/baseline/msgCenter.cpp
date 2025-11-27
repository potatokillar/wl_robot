#include "baseline.hpp"

MsgContext::MsgContext(u32 size) : size_(size)
{
    mutexTimeout_ = 200;  // 互斥量锁定最长200us
}

void MsgContext::Connect(const std::string& tag, std::size_t hash)
{
    if (msg_[tag].count(hash) == 0) msg_[tag].emplace(hash, size_);
}

/**
 * @description: 取消连接
 * @param tag
 * @param hash
 * @return {}
 */
void MsgContext::DisConnect(const std::string& tag, std::size_t hash) { msg_[tag].erase(hash); }

/**
 * @description: 非阻塞推入
 * @param tag
 * @param data
 * @return {}
 */
bool MsgContext::TryPush(const std::string& tag, const MsgType& data)
{
    std::unique_lock lock(mutex_, std::chrono::microseconds(mutexTimeout_));
    if (lock.owns_lock() == false) {
        LOG_DEBUG("push lock mutex fail {}", tag);
        return false;
    }
    return InnerPush(tag, data, false);
}

/**
 * @description: 强制塞入
 * @param tag
 * @param data
 * @return {}
 */
bool MsgContext::ForcePush(const std::string& tag, const MsgType& data)
{
    std::unique_lock lock(mutex_, std::chrono::microseconds(mutexTimeout_));
    if (lock.owns_lock() == false) {
        LOG_DEBUG("push lock mutex fail {}", tag);
        return false;
    }
    return InnerPush(tag, data, true);
}

bool MsgContext::Push(const std::string& tag, const MsgType& data, u32 ms)
{
    bool ret = false;
    std::unique_lock lock(mutex_);
    if (ms != 0) {
        while (FifoCanPush(tag) == false) {
            if (cvPush_[tag].wait_for(lock, std::chrono::milliseconds(ms)) == std::cv_status::timeout) {
                return false;
            }
        }
    } else {
        while (FifoCanPush(tag) == false) {
            cvPush_[tag].wait(lock);
        }
    }
    ret = InnerPush(tag, data, false);

    return ret;
}
/**
 * @description: 非阻塞取
 * @param tag
 * @param hash
 * @param flag
 * @param ms
 * @return {}
 */
std::optional<MsgType> MsgContext::TryPop(const std::string& tag, std::size_t hash)
{
    std::unique_lock lock(mutex_, std::chrono::microseconds(mutexTimeout_));
    if (lock.owns_lock() == false) {
        LOG_INFO("pop lock mutex fail {}", tag);
        return std::nullopt;
    }
    return InnerPop(tag, hash);
}
/**
 * @description: 阻塞获取一个数据
 * @param tag
 * @param hash
 * @param flag
 * @param ms
 * @return {}
 */
std::optional<MsgType> MsgContext::Pop(const std::string& tag, std::size_t hash, u32 ms)
{
    std::optional<MsgType> ret;
    std::unique_lock lock(mutex_);
    if (ms != 0) {
        while (FifoCanPop(tag, hash) == false) {
            if (cvPop_[tag].wait_for(lock, std::chrono::milliseconds(ms)) == std::cv_status::timeout) {
                return std::nullopt;
            }
        }
    } else {
        while (FifoCanPop(tag, hash) == false) {
            cvPop_[tag].wait(lock);
        }
    }
    ret = InnerPop(tag, hash);
    return ret;
}

/**
 * @description: 删除该tag下的所有数据，一般用于发送者
 * @param tag
 * @param flag
 * @return {}
 */
void MsgContext::Clear(const std::string& tag)
{
    std::unique_lock lock(mutex_);
    return InnerClear(tag);
}

/**
 * @description: 删除该tag下，该hash下的所有数据，一般用于接收者
 * @param tag
 * @param hash
 * @param flag
 * @return {}
 */
void MsgContext::Clear(const std::string& tag, std::size_t hash)
{
    std::unique_lock lock(mutex_);
    return InnerClear(tag, hash);
}

bool MsgContext::InnerPush(const std::string& tag, const MsgType& data, bool isForce)
{
    try {
        bool ret = true;
        if (isForce) {
            for (auto& v : msg_.at(tag)) {
                ret = v.second.push_force(data) && ret;
            }
        } else {
            for (auto& v : msg_.at(tag)) {
                ret = v.second.push(data) && ret;
            }
        }
        cvPop_[tag].notify_all();
        return ret;
    } catch (std::exception& e) {
        // LOG_DEBUG("InnerPush pop excep");
        return false;
    }
}

std::optional<MsgType> MsgContext::InnerPop(const std::string& tag, std::size_t hash)
{
    try {
        // auto t1 = TimerTools::GetNowTickUs();
        auto ret = msg_.at(tag).at(hash).pop();
        if (ret) {
            cvPush_[tag].notify_all();  // 通知push，写在这里，可以支持阻塞和非阻塞同时使用，否则阻塞可能收不到非阻塞塞入成功的消息
                                        /*             auto t2 = TimerTools::GetNowTickUs();
                                                    if (t2 - t1 > 50) {
                                                        LOG_DEBUG("InnerPop {}", t2 - t1);
                                                    } */
            return ret.value();
        }
    } catch (std::exception& e) {
        // LOG_DEBUG("inner pop excep");
        return std::nullopt;
    }
    return std::nullopt;
}

void MsgContext::InnerClear(const std::string& tag)
{
    if (msg_.count(tag) == 0) {
        return;
    }

    for (auto& v : msg_.at(tag)) {
        v.second.clear();
    }
}

void MsgContext::InnerClear(const std::string& tag, std::size_t hash)
{
    if ((msg_.count(tag) == 0) || (msg_.at(tag).count(hash) == 0)) {
        return;
    }

    msg_.at(tag).at(hash).clear();
}
/**
 * @description: 该消息可以被写入，即存在接收者，且接收者已经取走数据
 * @param tag
 * @return {}
 */
bool MsgContext::FifoCanPush(const std::string& tag)
{
    // 没有接收者connect，永远无法成功写入
    if (msg_.count(tag) == 0) {
        return false;
    } else {
        // 遍历接收者的fifo，若fifo满表示无法写入
        for (const auto& f : msg_.at(tag)) {
            if (f.second.full() == true) {
                return false;
            }
        }
        // 遍历完成，不存在无法写入的接收者
        return true;
    }
}

/**
 * @description: 消息可被读取
 * @param tag
 * @param hash
 * @return {}
 */
bool MsgContext::FifoCanPop(const std::string& tag, std::size_t hash)
{
    // 没有接收者connect，永远无法成功取出
    if ((msg_.count(tag) == 0) || (msg_.at(tag).count(hash) == 0)) {
        return false;
    } else {
        // 对于fifo有数据，返回true
        if (msg_.at(tag).at(hash).empty() == false) {
            return true;
        }
        return false;
    }
}

/////////////////////////////////////////////////////////////////////////////

MsgBase::MsgBase(u32 size) : ctx_(size) {}

bool MsgBase::TrySend(const std::string& tag, const MsgType& data) { return ctx_.TryPush(tag, data); }
bool MsgBase::ForceSend(const std::string& tag, const MsgType& data) { return ctx_.ForcePush(tag, data); }
bool MsgBase::BlockSend(const std::string& tag, const MsgType& data, u32 ms) { return ctx_.Push(tag, data, ms); }

std::optional<MsgType> MsgBase::TryRecv(const std::string& tag, std::size_t hash) { return ctx_.TryPop(tag, hash); }

std::optional<MsgType> MsgBase::BlockRecv(const std::string& tag, std::size_t hash, u32 ms) { return ctx_.Pop(tag, hash, ms); }

void MsgBase::Connect(const std::string& tag, std::size_t hash) { ctx_.Connect(tag, hash); }

void MsgBase::DisConnect(const std::string& tag, std::size_t hash) { ctx_.DisConnect(tag, hash); }

void MsgBase::Clear(const std::string& tag) { ctx_.Clear(tag); }

void MsgBase::Clear(const std::string& tag, std::size_t hash) { ctx_.Clear(tag, hash); }

#pragma once

#include <algorithm>
#include <any>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <deque>
#include <functional>
#include <iostream>
#include <map>
#include <mutex>
#include <queue>
#include <random>
#include <set>
#include <thread>
#include <type_traits>
#include <vector>

#include "cppType.hpp"
#include "robotLog.hpp"
#include "timerTools.hpp"

// C++没有环形队列 单独实现
// 仿标准库写法
template <typename T>
class fifo
{
public:
    fifo(u32 size)
    {
        if (size <= 1) {
            LOG_CRITICAL("fifo exception: size must > 1");
            throw std::invalid_argument("size must > 1");
        }
        for (u32 i = 0; i < size; i++) {
            T ele;  // 让fifo调用私有构造函数
            buf.push_back(ele);
        }
        // buf.resize(size);
        total = size;
    }

    void clear()
    {
        front = 0;
        rear = 0;
    }

    bool empty() const
    {
        if (front == rear) {
            return true;
        } else {
            return false;
        }
    }

    bool full() const
    {
        if (front == (rear + 1) % total) {
            return true;
        } else {
            return false;
        }
    }

    u32 size() const { return ((rear + total - front) % total); }

    bool push(T var)
    {
        if (full()) {
            return false;
        }

        buf[rear] = var;
        rear = (rear + 1) % total;
        return true;
    }

    bool push_force(T var)
    {
        if (full()) {
            pop();
        }

        buf[rear] = var;
        rear = (rear + 1) % total;
        return true;
    }

    std::optional<T> pop()
    {
        if (empty()) {
            return std::nullopt;
        }
        T var = buf[front];
        front = (front + 1) % total;
        return var;
    }

private:
    u32 front{0};
    u32 rear{0};
    u32 total;
    std::vector<T> buf;
};

class MsgType
{
public:
    /**
     * @description: 任意类型构造
     * @return {}
     */
    template <typename T>
    MsgType(T data)
    {
        msg_ = std::any(data);
    }

    template <typename T>
    T GetType() const
    {
        try {
            return std::any_cast<T>(msg_);
        } catch (std::exception& e) {
            LOG_CRITICAL("MsgType exception:{}, except type:{}, actual type:{}", e.what(), typeid(T).name(), msg_.type().name());
            throw e;
        }
    }

private:
    friend class fifo<MsgType>;
    MsgType() = default;

    std::any msg_;
};

class MsgContext
{
public:
    MsgContext(u32 size);
    void Connect(const std::string& tag, std::size_t hash);
    void DisConnect(const std::string& tag, std::size_t hash);

    bool TryPush(const std::string& tag, const MsgType& data);
    bool ForcePush(const std::string& tag, const MsgType& data);
    bool Push(const std::string& tag, const MsgType& data, u32 ms);

    std::optional<MsgType> TryPop(const std::string& tag, std::size_t hash);
    std::optional<MsgType> Pop(const std::string& tag, std::size_t hash, u32 ms);

    void Clear(const std::string& tag);
    void Clear(const std::string& tag, std::size_t hash);

private:
    bool InnerPush(const std::string& tag, const MsgType& data, bool isForce);
    std::optional<MsgType> InnerPop(const std::string& tag, std::size_t hash);
    void InnerClear(const std::string& tag);
    void InnerClear(const std::string& tag, std::size_t hash);

    bool FifoCanPush(const std::string& tag);
    bool FifoCanPop(const std::string& tag, std::size_t hash);

private:
    std::map<std::string, std::map<std::size_t, fifo<MsgType>>> msg_;
    u32 size_;
    std::timed_mutex mutex_;
    int mutexTimeout_;
    std::map<std::string, std::condition_variable_any> cvPush_;
    std::map<std::string, std::condition_variable_any> cvPop_;
};

// 基类框架，基本上仅仅是对MsgContext的封装
class MsgBase
{
public:
    MsgBase(u32 size);

    bool TrySend(const std::string& tag, const MsgType& data);
    bool ForceSend(const std::string& tag, const MsgType& data);
    bool BlockSend(const std::string& tag, const MsgType& data, u32 ms);
    std::optional<MsgType> TryRecv(const std::string& tag, std::size_t hash);
    std::optional<MsgType> BlockRecv(const std::string& tag, std::size_t hash, u32 ms);

    void Connect(const std::string& tag, std::size_t hash);
    void DisConnect(const std::string& tag, std::size_t hash);
    void Clear(const std::string& tag);
    void Clear(const std::string& tag, std::size_t hash);

private:
    MsgContext ctx_;  // fifo长度，默认非阻塞
};

// 快速发送框架
class MsgFast : public MsgBase
{
public:
    static MsgFast& GetInstance()
    {
        static MsgFast instance;
        return instance;
    }

private:
    MsgFast() : MsgBase(2) {}
};
inline MsgFast& GetMsgFast() { return MsgFast::GetInstance(); }

// RPC单例，主要是用于同一个RPC之间的信息交互
class MsgRpc : public MsgBase
{
public:
    static MsgRpc& GetInstance()
    {
        static MsgRpc instance;
        return instance;
    }

private:
    MsgRpc() : MsgBase(10) {}
};
inline MsgRpc& GetMsgRpc() { return MsgRpc::GetInstance(); }

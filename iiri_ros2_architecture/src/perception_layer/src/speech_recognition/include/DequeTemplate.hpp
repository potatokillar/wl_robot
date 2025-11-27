/*
 * @Author: 唐文浩
 * @Date: 2024-12-27
 * @LastEditors: 唐文浩
 * @LastEditTime: 2024-12-27
 * @Description:
 *
 * Copyright (c) 2022-2024 by 西湖大学智能产业研究院, All Rights Reserved.
 */
#pragma once
#include <condition_variable>
#include <deque>
#include <mutex>
#include <optional>

template <typename T>
class DequeTemplate
{
public:
    void Push(T element) // 在队列末尾插入一个元素
    {
        std::lock_guard<std::mutex> lock(_mtx);
        _data.push_back(element);
        _cond.notify_one(); // 唤醒一个等待的线程
    };

    T Pop() // 从队列前方弹出一个元素
    {
        std::unique_lock<std::mutex> lock(_mtx);
        _cond.wait(lock, [this]
                   { return !_data.empty(); }); // 等待直到deque不为空
        T ret = _data.front();
        _data.pop_front();
        return ret;
    };

    std::optional<T> PopWithTimeout(std::chrono::milliseconds timeout = std::chrono::milliseconds(-1))
    {
        std::unique_lock<std::mutex> lock(_mtx);

        // 等待直到deque不为空或者超时/中断
        if (_cond.wait_for(lock, timeout, [this]
                           { return !_data.empty(); }))
        {
            if (!_data.empty())
            {
                T ret = _data.front();
                _data.pop_front();
                return ret;
            }
        }

        // 如果是因为 _global_run 为 false 而唤醒，则返回 nullopt 表示没有读取到有效数据
        return std::nullopt;
    };

    bool IsEmpty() { return _data.empty(); };

    uint32_t Size()
    {
        std::lock_guard<std::mutex> lock(_mtx);
        return _data.size();
    };

    void Clear()
    {
        std::lock_guard<std::mutex> lock(_mtx);
        _data.clear();
    };

private:
    std::deque<T> _data;
    std::mutex _mtx;
    std::condition_variable _cond;
};
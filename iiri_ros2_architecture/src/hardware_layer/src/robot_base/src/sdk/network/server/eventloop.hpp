/*
 * @Author: 唐文浩
 * @Date: 2022-05-07
 * @LastEditors: 唐文浩
 * @LastEditTime: 2023-11-20
 * @Description: 事件循环
 *
 * Copyright (c) 2022-2022 by 西湖大学智能产业研究院, All Rights Reserved.
 */

#pragma once

#include <event.h>
#include <event2/bufferevent.h>
#include <event2/listener.h>

#include <functional>
#include <map>
#include <string>
#include <thread>
#include <vector>

// 超时事件
using EvtTimeoutCallbackType = std::function<void(void)>;

// 一个EventLoop对象只有一个event_base, 但不止一个event。
// 事件回调需要的参数
class EventLoop;
struct EventArgs
{
    EventLoop *ptr;
    std::string name;
};

// 每一个事件的信息
struct EventInfo
{
    struct event *evt{nullptr};
    EvtTimeoutCallbackType timeoutCb; // 超时回调
    EventArgs args;
};

/* 使用方法
 * 1，实例化一个EventLoop对象
 * 2，实例化一个TCP/UDP对象，EventLoop是其参数
 * 3，设置TCP/UDP对象的内容
 * 4，启动EventLoop的循环
 */

class EventLoop
{
public:
    EventLoop();
    virtual ~EventLoop();

    void Stop();
    void Loop(); // 循环事件

    bool AddTimeoutCb(const std::string &name, int ms, EvtTimeoutCallbackType func);
    bool RmvTimeoutCb(const std::string &name);

    // 超时事件
    static void TimeoutCb(evutil_socket_t fd, short what, void *thisPtr);
    void TimeoutCb(evutil_socket_t fd, short what, const std::string &name);

    struct event_base *evtBase_;
    std::map<std::string, EventInfo> evtMap_; // 事件的集合
};
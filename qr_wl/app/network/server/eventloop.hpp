

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

class EventLoop;
struct EventArgs
{
    EventLoop *ptr;
    std::string name;
};

struct EventInfo
{
    struct event *evt{nullptr};
    EvtTimeoutCallbackType timeoutCb;  // 超时回调
    EventArgs args;
};

class EventLoop
{
public:
    EventLoop();
    virtual ~EventLoop();

    void Stop();
    void Loop();

    bool AddTimeoutCb(const std::string &name, int ms, EvtTimeoutCallbackType func);
    bool RmvTimeoutCb(const std::string &name);

    static void TimeoutCb(evutil_socket_t fd, short what, void *thisPtr);
    void TimeoutCb(evutil_socket_t fd, short what, const std::string &name);

    struct event_base *evtBase_;
    std::map<std::string, EventInfo> evtMap_;  // 事件的集合
};
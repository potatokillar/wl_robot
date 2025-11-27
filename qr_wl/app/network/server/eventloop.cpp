
#include "eventloop.hpp"

#include "event2/thread.h"

EventLoop::EventLoop()
{
    evthread_use_pthreads();
    evtBase_ = event_base_new();
}

EventLoop::~EventLoop() { Stop(); }

void EventLoop::Stop() { event_base_loopexit(evtBase_, NULL); }

void EventLoop::Loop() { event_base_dispatch(evtBase_); }

bool EventLoop::AddTimeoutCb(const std::string &name, int ms, EvtTimeoutCallbackType func)
{
    if (evtMap_.count(name) > 0) {
        return false;
    }
    evtMap_[name].timeoutCb = func;
    evtMap_[name].args.ptr = this;
    evtMap_[name].args.name = name;

    struct timeval tv;
    tv.tv_sec = ms / 1000;
    tv.tv_usec = ms % 1000 * 1000;
    evtMap_[name].evt = event_new(evtBase_, -1, EV_PERSIST, &EventLoop::TimeoutCb, &evtMap_[name].args);
    int ret = event_add(evtMap_[name].evt, &tv);
    if (ret == 0) {
        return true;
    }
    return false;
}
bool EventLoop::RmvTimeoutCb(const std::string &name)
{
    if (evtMap_.count(name)) {
        event_remove_timer(evtMap_[name].evt);
        evtMap_.erase(name);
    } else {
        return false;
    }
    return true;
}

void EventLoop::TimeoutCb(evutil_socket_t fd, short what, void *ptr)
{
    EventArgs *arg = (EventArgs *)ptr;
    arg->ptr->TimeoutCb(fd, what, arg->name);
}
void EventLoop::TimeoutCb(evutil_socket_t fd, short what, const std::string &name)
{
    (void)fd;
    (void)what;
    if (evtMap_[name].timeoutCb) {
        evtMap_[name].timeoutCb();
    }
}
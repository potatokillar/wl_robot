
#pragma once

#include <functional>
#include <memory>
#include <string>

#include "eventloop.hpp"
#include "netType.hpp"

using MulticastMsgCallbackType = std::function<void(const NetClientInfo& info, const std::vector<uint8_t>& data)>;

class Multicast
{
public:
    Multicast(std::shared_ptr<EventLoop> event, const std::string& ipv4, int port, const std::string& name);
    void Start();
    void SetReadCallback(MulticastMsgCallbackType func) { ReadFunc_ = func; }
    bool Send(const std::string& ip, u16 port, const std::vector<uint8_t>& data);

private:
    // 读事件
    static void ReadCb(evutil_socket_t fd, short what, void* ptr);
    void ReadCb(evutil_socket_t fd, short what);

private:
    std::shared_ptr<EventLoop> loop_;
    int sockFd_;
    int port_;
    std::string ipv4_;
    std::string name_;
    struct event* evt_{nullptr};
    MulticastMsgCallbackType ReadFunc_;
};
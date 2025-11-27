
#pragma once

#include <boost/asio.hpp>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "baseline.hpp"
#include "eventloop.hpp"
#include "netType.hpp"

constexpr int MAX_UDP_RX = 1472;

using UdpMsgCallbackType = std::function<void(const NetClientInfo &info, const std::vector<uint8_t> &data)>;

class UdpServer
{
public:
    UdpServer(std::shared_ptr<EventLoop> loop, int port, const std::string &name);
    ~UdpServer();

    void Start();
    void Stop();

    std::string GetServerName() const { return name_; }

    void SetReadCallback(UdpMsgCallbackType func) { ReadFunc_ = func; }
    bool Send(const std::string &ip, u16 port, const std::vector<uint8_t> &data);

private:
    static void ReadCb(evutil_socket_t fd, short what, void *ptr);
    void ReadCb(evutil_socket_t fd, short what);

private:
    std::shared_ptr<EventLoop> loop_;
    int port_;
    const std::string name_;

private:
    struct event *evt{nullptr};
    const int MAX_CONNECT = 5;

    UdpMsgCallbackType ReadFunc_;
    uint8_t rxbuf_[MAX_UDP_RX]{0};

    std::vector<uint8_t> rxData_;
    int fd_{-1};
};

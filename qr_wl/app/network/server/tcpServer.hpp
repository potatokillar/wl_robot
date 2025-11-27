
#pragma once

#include <functional>
#include <memory>
#include <string>

#include "eventloop.hpp"
#include "netType.hpp"
using TcpMsgCallbackType = std::function<void(struct bufferevent *bev)>;
using TcpTimeoutCallbackType = std::function<void(evutil_socket_t fd, short events)>;

struct ClientInfo
{
    struct bufferevent *bev{nullptr};
    std::string ipstr;
    int port;
    bool IsMe(struct bufferevent *inbev) { return inbev == bev; }
};

class TcpServer
{
public:
    TcpServer(std::shared_ptr<EventLoop> loop, int port, const std::string &name);
    ~TcpServer();

    void Start();
    void Stop();

    void Write(const uint8_t *data, uint16_t len);
    bool Send(const std::string &ip, u16 port, const std::vector<uint8_t> &data);
    bool Send(const std::vector<uint8_t> &data);

    std::string GetServerName() const { return name_; }

    void SetReadCallback(TcpMsgCallbackType func) { ReadFunc_ = func; }
    void SetTimeoutCallback(int ms, TcpTimeoutCallbackType TimeoutFunc);

private:
    static void ReadCb(struct bufferevent *bev, void *ptr);

    static void EventCb(struct bufferevent *bev, short what, void *ptr);
    void EventCb(struct bufferevent *bev, short what);

    static void TimeoutCb(evutil_socket_t fd, short events, void *ptr);

    static void ListenCb(struct evconnlistener *, evutil_socket_t, struct sockaddr *, int socklen, void *);
    void ListenCb(struct evconnlistener *, evutil_socket_t, struct sockaddr *, int socklen);

private:
    std::shared_ptr<EventLoop> loop_;
    int port_;
    const std::string name_;

private:
    struct evconnlistener *listener{nullptr};
    struct event *evt;
    const int MAX_CONNECT = 5;

    TcpMsgCallbackType ReadFunc_;
    TcpTimeoutCallbackType TimeoutFunc_;

    std::vector<ClientInfo> clientSet_;
};

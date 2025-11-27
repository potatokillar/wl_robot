/*
 * @Author: 唐文浩
 * @Date: 2022-03-14
 * @LastEditors: 唐文浩
 * @LastEditTime: 2025-02-18
 * @Description: TCP小型服务器，相比原先的，采用了libevent架构
 *  2022-03-14 增加多客户端连接
 * Copyright (c) 2022-2022 by 西湖大学智能产业研究院, All Rights Reserved.
 */
#pragma once

#include <functional>
#include <memory>
#include <string>

#include "eventloop.hpp"
#include "netType.hpp"
using TcpMsgCallbackType = std::function<void(struct bufferevent *bev)>;
using TcpTimeoutCallbackType = std::function<void(evutil_socket_t fd, short events)>;

// TCP为长连接，需要保存客户端的数据
struct ClientInfo
{
    struct bufferevent *bev{nullptr};
    std::string ipstr;
    int port;
    bool IsMe(struct bufferevent *inbev) { return inbev == bev; }
};

// TCP服务器，一个类实例监控一个地址
class TcpServer
{
public:
    TcpServer(std::shared_ptr<EventLoop> loop, int port, const std::string &name);
    ~TcpServer();

    void Start();
    void Stop();

    // 主动写数据，没有写事件
    void Write(const uint8_t *data, uint16_t len);
    bool Send(const std::string &ip, uint16_t port, const std::vector<uint8_t> &data);
    bool Send(const std::vector<uint8_t> &data);

    // 获取名字
    std::string GetServerName() const { return name_; }

    void SetReadCallback(TcpMsgCallbackType func) { ReadFunc_ = func; }
    void SetTimeoutCallback(int ms, TcpTimeoutCallbackType TimeoutFunc);

private:
    // 读事件
    static void ReadCb(struct bufferevent *bev, void *ptr);

    // 事件回调
    static void EventCb(struct bufferevent *bev, short what, void *ptr);
    void EventCb(struct bufferevent *bev, short what);

    // 超时回调
    static void TimeoutCb(evutil_socket_t fd, short events, void *ptr);

    // 监听连接事件
    static void ListenCb(struct evconnlistener *, evutil_socket_t, struct sockaddr *, int socklen, void *);
    void ListenCb(struct evconnlistener *, evutil_socket_t, struct sockaddr *, int socklen);

private:
    std::shared_ptr<EventLoop> loop_; // 事件循环体
    int port_;                        // 监听端口
    const std::string name_;          // 本服务器的名字

private:
    struct evconnlistener *listener{nullptr}; // 监听的事件
    struct event *evt;
    const int MAX_CONNECT = 5;
    // 回调函数
    TcpMsgCallbackType ReadFunc_;
    TcpTimeoutCallbackType TimeoutFunc_;

    // 每个服务端可以有多个客户端，但共享处理逻辑
    std::vector<ClientInfo> clientSet_;
};

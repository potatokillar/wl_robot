/*
 * @Author: 唐文浩
 * @Date: 2022-05-07
 * @LastEditors: 唐文浩
 * @LastEditTime: 2025-02-19
 * @Description: UDP小型服务器，相比原先的，采用了libevent架构
 *
 * Copyright (c) 2022-2022 by 西湖大学智能产业研究院, All Rights Reserved.
 */
#pragma once

#include <boost/asio.hpp>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "eventloop.hpp"
#include "netType.hpp"

// 最大UDP数据包长度，MTU(1500) - IP头(20) -UDP头(8)
constexpr int MAX_UDP_RX = 1472;

/**
 * @description: UDP回调函数定义
 * @param &name 调用回调函数的UDP实例名称，构造udpServer时提供的，提供该值可以允许多个udp实例使用同一个回调
 * @param &data
 * @return {}
 */
using UdpMsgCallbackType = std::function<void(const NetClientInfo &info, const std::vector<uint8_t> &data)>;
// UDP服务器，一个类实例监控一个地址
class UdpServer
{
public:
    UdpServer(std::shared_ptr<EventLoop> loop, int port, const std::string &name);
    ~UdpServer();

    void Start();
    void Stop();

    // 获取名字
    std::string GetServerName() const { return name_; }

    void SetReadCallback(UdpMsgCallbackType func) { ReadFunc_ = func; }
    bool Send(const std::string &ip, uint16_t port, const std::vector<uint8_t> &data);

private:
    // 读事件
    static void ReadCb(evutil_socket_t fd, short what, void *ptr);
    void ReadCb(evutil_socket_t fd, short what);

private:
    std::shared_ptr<EventLoop> loop_; // 事件循环体
    int port_;                        // 监听端口
    const std::string name_;          // 本服务器的名字

private:
    struct event *evt{nullptr};
    const int MAX_CONNECT = 5;
    // 回调函数
    UdpMsgCallbackType ReadFunc_;
    uint8_t rxbuf_[MAX_UDP_RX]{0};

    std::vector<uint8_t> rxData_;
    int fd_{-1};

    boost::asio::thread_pool threadPool_{2}; // 2个
};

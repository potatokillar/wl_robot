/*
 * @Author: 唐文浩
 * @Date: 2022-05-07
 * @LastEditors: 唐文浩
 * @LastEditTime: 2025-02-19
 * @Description: 多播服务器
 *
 * Copyright (c) 2022-2022 by 西湖大学智能产业研究院, All Rights Reserved.
 */
#pragma once

#include <functional>
#include <memory>
#include <string>

#include "eventloop.hpp"
#include "netType.hpp"

using MulticastMsgCallbackType = std::function<void(const NetClientInfo &info, const std::vector<uint8_t> &data)>;

class Multicast
{
public:
    Multicast(std::shared_ptr<EventLoop> event, const std::string &ipv4, int port, const std::string &name);
    void Start();
    void SetReadCallback(MulticastMsgCallbackType func) { ReadFunc_ = func; }
    bool Send(const std::string &ip, uint16_t port, const std::vector<uint8_t> &data);

private:
    // 读事件
    static void ReadCb(evutil_socket_t fd, short what, void *ptr);
    void ReadCb(evutil_socket_t fd, short what);

private:
    std::shared_ptr<EventLoop> loop_;
    int sockFd_;
    int port_;         // 端口
    std::string ipv4_; // 多播组
    std::string name_;
    struct event *evt_{nullptr};
    // 回调函数
    MulticastMsgCallbackType ReadFunc_;
};
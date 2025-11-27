/*
 * @Author: 唐文浩-0036
 * @Date: 2023-07-10
 * @LastEditors: 唐文浩
 * @LastEditTime: 2025-02-19
 * @Description:
 *
 * Copyright (c) 2022-2023 by 西湖大学智能产业研究院, All Rights Reserved.
 */
#include "multicast.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <strings.h>
#include <sys/socket.h>

using namespace std;

Multicast::Multicast(std::shared_ptr<EventLoop> event, const std::string &ipv4, int port, const std::string &name)
{
    loop_ = event;
    ipv4_ = ipv4;
    port_ = port;
    name_ = name;
}

void Multicast::Start()
{
    sockFd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (sockFd_ < 0)
    {
        // LOG_WARN_SDK("multicast socket error");
    }

    // 设置多播监控的端口
    struct sockaddr_in localaddr;
    bzero(&localaddr, sizeof(localaddr));
    localaddr.sin_family = AF_INET;
    localaddr.sin_port = htons(port_);
    localaddr.sin_addr.s_addr = htonl(INADDR_ANY);
    if (::bind(sockFd_, (struct sockaddr *)&localaddr, sizeof(struct sockaddr_in)) < 0)
    {
        //  LOG_WARN_SDK("multicast bind error");
    }

    // 设置多播的TTL值，1表示只在同一路由下的局域网进行转播
    int ttl = 5;
    if (setsockopt(sockFd_, IPPROTO_IP, IP_MULTICAST_TTL, &ttl, sizeof(ttl)) < 0)
    {
        // LOG_WARN_SDK("multicast set ttl error");
    }

    // 设置是否发送到本地环回接口
    int loop = 0;
    if (setsockopt(sockFd_, IPPROTO_IP, IP_MULTICAST_LOOP, &loop, sizeof(loop)) < 0)
    {
        //  LOG_WARN_SDK("multicast set loop error");
    }

    // 加入多播组
    struct ip_mreq mreq;
    bzero(&mreq, sizeof(mreq));
    inet_pton(AF_INET, ipv4_.c_str(), &mreq.imr_multiaddr); // 多播组的IP
    // mreq.imr_interface.s_addr = htonl(INADDR_ANY);  //本机的默认接口IP,本机的随机IP
    if (setsockopt(sockFd_, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq)) < 0)
    {
        // LOG_WARN_SDK("multicast add error");
    }

    // 添加到事件
    evt_ = event_new(loop_->evtBase_, sockFd_, EV_READ | EV_PERSIST, &Multicast::ReadCb, this);
    event_add(evt_, 0);
    // LOG_INFO_SDK("findserver ok");
}

// 读回调
void Multicast::ReadCb(evutil_socket_t fd, short what, void *ptr)
{
    Multicast *myClass = (Multicast *)ptr;
    myClass->ReadCb(fd, what);
}

void Multicast::ReadCb(evutil_socket_t fd, short what)
{
    (void)what;
    struct sockaddr_in recvAddr; // 存放对端数据
    socklen_t socklen = sizeof(struct sockaddr);

    char rxbuf[100];
    // 接收数据，UDP发现的数据并不多，一次可接收完毕
    int rxLen = recvfrom(fd, &rxbuf, sizeof(rxbuf), 0, (struct sockaddr *)&recvAddr, &socklen);
    if (rxLen == -1)
    {
        // LOG_WARN("find server recv error");
        return;
    }

    // 调用用户定义的回调
    if (ReadFunc_)
    {
        auto [ret, ip, port] = Sockaddr2String(recvAddr);
        if (ret == true)
        {
            vector<uint8_t> data(rxbuf, rxbuf + rxLen);
            NetClientInfo info;
            info.serverName = name_;
            info.clientIp = ip;
            info.clientPort = port;
            ReadFunc_(info, data);
        }
    }
}

/**
 * @description: 发送数据
 * @param &addr 发送的客户端地址，在接收回调中会提供
 * @param &data
 * @return {}
 */
bool Multicast::Send(const std::string &ip, uint16_t port, const std::vector<uint8_t> &data)
{
    auto [ret, sockaddr] = String2Sockaddr(ip, port);

    if (ret == true)
    {
        if (sendto(sockFd_, data.data(), data.size(), 0, (struct sockaddr *)&sockaddr, sizeof(struct sockaddr)) >= 0)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    else
    {
        return false;
    }
}

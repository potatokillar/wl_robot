/*
 * @Author: 唐文浩
 * @Date: 2022-05-07
 * @LastEditors: 唐文浩
 * @LastEditTime: 2025-02-18
 * @Description:
 *
 * Copyright (c) 2022-2022 by 西湖大学智能产业研究院, All Rights Reserved.
 */
#include "udpServer.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <signal.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <cstring>

using namespace std;

UdpServer::UdpServer(std::shared_ptr<EventLoop> loop, int port, const std::string &name) : loop_(loop), port_(port), name_(name) {}

UdpServer::~UdpServer() {}

void UdpServer::Start()
{
    fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in server_addr;
    // 填充sockaddr_in结构
    bzero(&server_addr, sizeof(struct sockaddr_in));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    server_addr.sin_port = htons(port_);

    if (bind(fd_, (struct sockaddr *)&server_addr, sizeof(server_addr)))
    {
        // LOG_WARN_SDK("bind error");
    }

    evt = event_new(loop_->evtBase_, fd_, EV_READ | EV_PERSIST, &UdpServer::ReadCb, this);
    event_add(evt, 0);
}

/****************************************************************
 * 功能：UDP接收数据回调，不支持接收大于1472字节的数据
 * 输入：@1
 * 输出：@1
 ****************************************************************/
void UdpServer::ReadCb(evutil_socket_t fd, short what, void *ptr)
{
    UdpServer *myClass = (UdpServer *)ptr;
    myClass->ReadCb(fd, what);
}
void UdpServer::ReadCb(evutil_socket_t fd, short what)
{
    (void)what;
    socklen_t socklen = sizeof(struct sockaddr);
    NetClientInfo info;
    info.serverName = name_;

    struct sockaddr_in addr;
    // 接收数据，UDP发现的数据并不多，一次可接收完毕
    int len = recvfrom(fd, rxbuf_, sizeof(rxbuf_), 0, (struct sockaddr *)&addr, &socklen);
    if (len == -1)
    {
        // LOG_WARN_SDK("find server recv error");
        return;
    }

    // 调用用户定义的回调
    if (ReadFunc_)
    {
        vector<uint8_t> data(rxbuf_, rxbuf_ + len);
        NetClientInfo info;
        auto [ret, ip, port] = Sockaddr2String(addr);
        if (ret == true)
        {
            info.clientIp = ip;
            info.clientPort = port;
            info.serverName = name_;
            //  ReadFunc_(info, data);
            boost::asio::post(threadPool_, [this, info, data]()
                              { this->ReadFunc_(info, data); });
        }
    }
}

/**
 * @description: 发送数据
 * @param &addr 发送的客户端地址，在接收回调中会提供
 * @param &data
 * @return {}
 */
bool UdpServer::Send(const std::string &ip, uint16_t port, const std::vector<uint8_t> &data)
{
    auto [ret, sockaddr] = String2Sockaddr(ip, port);

    if (ret == true)
    {
        if (sendto(fd_, data.data(), data.size(), 0, (struct sockaddr *)&sockaddr, sizeof(struct sockaddr)) >= 0)
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

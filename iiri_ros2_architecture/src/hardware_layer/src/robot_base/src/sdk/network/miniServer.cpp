/*
 * @Author: 唐文浩-0036
 * @Date: 2023-07-10
 * @LastEditors: 唐文浩
 * @LastEditTime: 2025-02-19
 * @Description:
 *
 * Copyright (c) 2022-2023 by 西湖大学智能产业研究院, All Rights Reserved.
 */
#include "miniServer.hpp"

#include <iostream>
using namespace std;

MiniServer::MiniServer() { evtloop_ = make_shared<EventLoop>(); }

MiniServer::~MiniServer() { evtloop_->Stop(); }

/**
 * @description: 添加udp服务器，不同种类的也不许重名，同一种类的同名会覆盖
 * @param &name
 * @param port
 * @return {}
 */
bool MiniServer::AddUdpServer(const std::string &name, uint16_t port)
{
    // 不同种类的不允许重名
    if (tcpSer_.count(name))
    {
        return false;
    }
    if (mcastSer_.count(name))
    {
        return false;
    }
    // 创建一个udp服务器
    udpSer_[name] = make_shared<UdpServer>(evtloop_, port, name);
    udpSer_[name]->SetReadCallback([this](const NetClientInfo &info, const vector<uint8_t> &data)
                                   { this->UdpRecv(info, data); });
    udpSer_[name]->Start();
    return true;
}

bool MiniServer::AddTcpServer(const std::string &name, uint16_t port)
{
    if (udpSer_.count(name))
    {
        return false;
    }
    if (mcastSer_.count(name))
    {
        return false;
    }
    tcpSer_[name] = make_shared<TcpServer>(evtloop_, port, name);
    tcpSer_[name]->Start();
    return false;
}

bool MiniServer::AddMcastServer(const std::string &name, const std::string &ip, uint16_t port)
{
    if (tcpSer_.count(name))
    {
        return false;
    }
    if (udpSer_.count(name))
    {
        return false;
    }
    mcastSer_[name] = make_shared<Multicast>(evtloop_, ip, port, name);
    // 多播本质也是UDP
    mcastSer_[name]->SetReadCallback([this](const NetClientInfo &info, const vector<uint8_t> &data)
                                     { this->UdpRecv(info, data); });
    mcastSer_[name]->Start();
    return true;
}

void MiniServer::Loop()
{
    std::cout << "loop" << std::endl;
    evtloop_->Loop();
}

/**
 * @description: 注册SDK协议解析回调
 * @param func
 * @return {}
 */
void MiniServer::SetSdkParse(SdkDataParseType func) { sdkParseFunc_ = func; }

/**
 * @description: 注册原始数据的回调
 * @param &name
 * @param func
 * @return {}
 */
void MiniServer::SetRawParse(std::string &name, RawDataParseType func) { rawParseFunc_[name] = func; }

void MiniServer::UdpRecv(const NetClientInfo &info, const vector<uint8_t> &data)
{
    // 若注册了原始数据解析，则不再进行sdk数据解析
    if (rawParseFunc_.count(info.serverName))
    {
        rawParseFunc_[info.serverName](info, data);
        return;
    }

    // 解析出头和数据包
    ProtocolParse rxData(data);

    if (rxData.Empty() == false)
    {
        sdkParseFunc_(info, rxData);
    }
}

/**
 * @description: 发送数据
 * @param &info
 * @param &data
 * @return {}
 */
bool MiniServer::Send(const NetClientInfo &info, const std::vector<uint8_t> &data)
{
    auto name = info.serverName;
    if (udpSer_.count(name))
    {
        return udpSer_[name]->Send(info.clientIp, info.clientPort, data);
    }
    if (tcpSer_.count(name))
    {
        return tcpSer_[name]->Send(info.clientIp, info.clientPort, data);
    }
    if (mcastSer_.count(name))
    {
        return mcastSer_[name]->Send(info.clientIp, info.clientPort, data);
    }
    return false;
}
bool MiniServer::Send(const NetClientInfo &info, const ProtocolConstruct &data) { return Send(info, data.GetData()); }

/**
 * @description: 发送给所有在线的客户端，只支持能保持长连接的协议，目前只有tcp
 * @param &name
 * @param &data
 * @return {}
 */
bool MiniServer::SendAll(const std::string &name, const std::vector<uint8_t> &data)
{
    if (tcpSer_.count(name))
    {
        return tcpSer_[name]->Send(data);
    }
    return false;
}

bool MiniServer::AddTimeoutCallback(const std::string &name, int ms, EvtTimeoutCallbackType TimeoutFunc)
{
    return evtloop_->AddTimeoutCb(name, ms, TimeoutFunc);
}

bool MiniServer::RmvTimeoutCb(const std::string &name) { return evtloop_->RmvTimeoutCb(name); }
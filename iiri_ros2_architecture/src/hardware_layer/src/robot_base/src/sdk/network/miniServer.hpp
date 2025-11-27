/*
 * @Author: 唐文浩-0036
 * @Date: 2023-07-10
 * @LastEditors: 唐文浩
 * @LastEditTime: 2025-02-19
 * @Description: 协议解析服务器，只支持智研院
 *
 * Copyright (c) 2022-2023 by 西湖大学智能产业研究院, All Rights Reserved.
 */
#pragma once
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "eventloop.hpp"
#include "multicast.hpp"
#include "netType.hpp"
#include "protocol.hpp"
#include "tcpServer.hpp"
#include "udpServer.hpp"

// 智研院SDK协议解析回调，此处将不区分数据是来自udp, tcp还是multicast，或将来的其他
// 只根据报文选项和额外选项进行区分回调
// 支持多个应用同时解析同一种报文，但不建议这么做
// 非阻塞回调
using SdkDataParseType = std::function<void(const NetClientInfo &info, const ProtocolParse &data)>;

// 原始数据解析，可以支持非智研院的协议，注册后，将屏蔽智研院的协议解析
using RawDataParseType = std::function<void(const NetClientInfo &info, const std::vector<uint8_t> &data)>;

class MiniServer
{
public:
    static MiniServer &GetInstance()
    {
        static MiniServer instance;
        return instance;
    }

    bool AddUdpServer(const std::string &name, uint16_t port);
    bool AddTcpServer(const std::string &name, uint16_t port);
    bool AddMcastServer(const std::string &name, const std::string &ip, uint16_t port);

    void SetSdkParse(SdkDataParseType func);
    void SetRawParse(std::string &name, RawDataParseType func);
    bool AddTimeoutCallback(const std::string &name, int ms, EvtTimeoutCallbackType TimeoutFunc);
    bool RmvTimeoutCb(const std::string &name);

    void Loop();

    bool Send(const NetClientInfo &info, const std::vector<uint8_t> &data);
    bool Send(const NetClientInfo &info, const ProtocolConstruct &data);

    bool SendAll(const std::string &name, const std::vector<uint8_t> &data);

private:
    MiniServer();
    virtual ~MiniServer();
    void UdpRecv(const NetClientInfo &info, const std::vector<uint8_t> &data);

private:
    std::shared_ptr<EventLoop> evtloop_; // 同一个mini用同一个事件循环
    std::map<std::string, std::shared_ptr<UdpServer>> udpSer_;
    std::map<std::string, std::shared_ptr<TcpServer>> tcpSer_;
    std::map<std::string, std::shared_ptr<Multicast>> mcastSer_;

    // 在这一层，能解析的数据要么是sdk协议规定的，要么是第三方原始数据
    SdkDataParseType sdkParseFunc_;
    std::map<std::string, RawDataParseType> rawParseFunc_;
};

inline MiniServer &GetMiniServer() { return MiniServer::GetInstance(); }

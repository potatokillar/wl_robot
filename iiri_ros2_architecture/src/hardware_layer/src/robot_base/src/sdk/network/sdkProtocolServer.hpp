/*
 * @Author: 唐文浩-0036
 * @Date: 2023-07-10
 * @LastEditors: 唐文浩
 * @LastEditTime: 2025-02-19
 * @Description: SDK协议服务端，智研院协议封包解包之后的一些通用处理
 *
 * Copyright (c) 2022-2023 by 西湖大学智能产业研究院, All Rights Reserved.
 */
#pragma once
#include <any>
#include <iomanip>
#include <set>

#include "../../../include/robot_base/timer_tools.hpp"
#include "miniServer.hpp"
#include "nlohmann/json.hpp"

using RpcServerCallbackType = std::function<nlohmann::json(nlohmann::json)>;
using BinCallbackType = std::function<std::vector<uint8_t>(const NetClientInfo &, const std::vector<uint8_t> &)>;

nlohmann::json ErrState2json(RetState sta);

// 订阅上报
struct SdkSubscribeFuncType
{
    uint32_t ms;                     // 默认的上报时间
    std::set<NetClientInfo> clients; // 订阅该函数的客户端
    std::function<std::pair<bool, nlohmann::json>()> func;
    std::string nameSpace;
    std::string method;
    bool addNew{false}; // 新客户端加入
    void Send();
    uint64_t lastPub_;
};

struct SubscribeInfo
{
    uint32_t lost;              // 心跳丢失次数，超过一定次数表明掉线
    std::set<std::string> keys; // 该客户端订阅的内容
};

class SdkProtocolServer
{
public:
    SdkProtocolServer();
    void JsonRecv(const NetClientInfo &info, const ProtocolParse &rxData);
    void JsonSend(const NetClientInfo &info, const nlohmann::json &method);

    void BinRecv(const NetClientInfo &info, const ProtocolParse &rxData);
    bool BinSend(const NetClientInfo &info, const std::vector<BinPackConstruct> &data);
    bool BinSend(const NetClientInfo &info, const BinPackConstruct &data);

    bool RawSend(const NetClientInfo &info, uint8_t opt, uint16_t extPot, const std::vector<uint8_t> &data);

    void AddMethodCall(const std::string &nameSpace, const std::string &method, RpcServerCallbackType func);
    void AddBinCall(const std::string &method, BinCallbackType func);
    void AddSubscribeCall(const std::string &nameSpace, const std::string &name, int ms, std::function<std::pair<bool, nlohmann::json>()> func);
    void AddTimeoutCall(const std::string &name, int ms, EvtTimeoutCallbackType func);

    NetClientInfo GetClientInfo();

private:
    void ParseRecv(const NetClientInfo &info, const ProtocolParse &rxData);
    void RmvSubscribeClient(const NetClientInfo &client);
    nlohmann::json Subscribe(const nlohmann::json &params);
    nlohmann::json Unsubscribe(const nlohmann::json &params);
    nlohmann::json RxAreYouOk(const nlohmann::json &params);
    void CheckClientOnlineStatus();

private:
    std::map<std::string, std::map<std::string, RpcServerCallbackType>> methodFunc_; // sdk json回复函数
    std::map<std::string, BinCallbackType> binFunc_;                                 // sdk 二进制回复函数

    NetClientInfo curClient_;                                                          // 当前客户端信息
    std::vector<NetClientInfo> clients_;                                               // 所有客户端信息
    std::vector<std::string> traceFilter_;                                             // LOG_TRACE打印过滤列表
    std::map<std::string, std::map<std::string, SdkSubscribeFuncType>> subscribeInfo_; // 订阅函数和客户端信息
};

inline std::string Double2Json(double number, size_t width = 6)
{
    std::stringstream ss;
    ss << std::fixed << std::setprecision(width) << number;
    return ss.str();
};

inline double Json2Double(nlohmann::json data) { return stod(std::string(data)); };

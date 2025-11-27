/*
 * @Author: 唐文浩-0036
 * @Date: 2023-07-10
 * @LastEditors: 唐文浩
 * @LastEditTime: 2025-02-19
 * @Description:
 *
 * Copyright (c) 2022-2023 by 西湖大学智能产业研究院, All Rights Reserved.
 */
#include "sdkProtocolServer.hpp"

#include <iostream>

using namespace std;
using namespace ::nlohmann;

/**
 * @description: 枚举转字符串，常量
 * @return {}
 */
static const map<RetState, string> RETSTATE_STRING = {
    {RetState::ok, "ok"},
    {RetState::netErr, "netErr"},
    {RetState::outRange, "outRange"},
    {RetState::timeout, "timeout"},
    {RetState::noSupport, "noSupport"},
    {RetState::parseErr, "parseErr"},
    {RetState::noUpdate, "noUpdate"},
    {RetState::busy, "busy"},
    {RetState::interrupt, "interrupt"},
    {RetState::noExist, "noExist"},
    {RetState::error, "error"},
};
nlohmann::json ErrState2json(RetState sta)
{
    if (RETSTATE_STRING.count(sta))
    {
        return RETSTATE_STRING.at(sta);
    }

    // LOG_ERROR_SDK("RetState2json error, please add: {}", Enum2Num(sta));
    return "error";
}

/**
 * @description: 订阅结构体的发送函数
 * @param isForce 强制发送
 * @return {}
 */
void SdkSubscribeFuncType::Send()
{
    auto bodyData = func();
    // 由业务控制是否上报该值
    if ((bodyData.first == true) || (addNew))
    {
        json jSend;
        jSend[nameSpace]["method"] = method;
        jSend[nameSpace]["result"] = bodyData.second;

        string tx = nlohmann::to_string(jSend) + '\0';

        ProtocolConstruct txData((uint8_t *)tx.c_str(), tx.size());
        txData.SetOption(0, 0); // 字符串为自定义字符串
        for (const auto &client : clients)
        {
            GetMiniServer().Send(client, txData);
        }

        if ((clients.empty() == false) && (TimerTools::GetNowTickMs() - lastPub_ > 1000))
        {
            //  LOG_TRACE_SDK("subscribe pub: {}", tx);
            lastPub_ = TimerTools::GetNowTickMs(); // 减少打印频率
        }
        addNew = false;
    }
}

SdkProtocolServer::SdkProtocolServer()
{
    GetMiniServer().AddUdpServer("udp-rpc", 20333);
    GetMiniServer().AddMcastServer("mcast", "224.0.0.88", 20444);

    GetMiniServer().SetSdkParse([this](const NetClientInfo &info, const ProtocolParse &data)
                                { this->ParseRecv(info, data); });

    traceFilter_.push_back("AreYouOk");
    traceFilter_.push_back("FindDevice");

    // 自行处理订阅有关的信息
    AddMethodCall("base", "Subscribe", [this](json params) -> json
                  { return this->Subscribe(params); });
    AddMethodCall("base", "Unsubscribe", [this](json params) -> json
                  { return this->Unsubscribe(params); });

    // 心跳
    AddMethodCall("device", "AreYouOk", [this](json params) -> json
                  { return this->RxAreYouOk(params); });
    AddTimeoutCall("CheckClientState", 100, [this]()
                   { this->CheckClientOnlineStatus(); });

    // 兼容性处理，心跳包从v2.4开始，从device空间移动到base空间
    AddMethodCall("base", "AreYouOk", [this](json params) -> json
                  { return this->RxAreYouOk(params); });
};

/**
 * @description: 数据接收解析，到这里的数据，sdk头校验已经通过
 * @param &info
 * @param &rxData
 * @return {}
 */
void SdkProtocolServer::ParseRecv(const NetClientInfo &info, const ProtocolParse &rxData)
{
    curClient_ = info;
    if (rxData.GetOption() == 0)
    {
        JsonRecv(info, rxData);
    }
    if ((rxData.GetOption() == 1) && (rxData.GetExtOption() == 2))
    {
        BinRecv(info, rxData);
    }
    curClient_ = {}; // 仅在回调函数中有效
}

/**
 * @description: 协议是json数据类型，进行解析
 * @param &info
 * @param &rxData
 * @return {}
 */
void SdkProtocolServer::JsonRecv(const NetClientInfo &info, const ProtocolParse &rxData)
{
    // 本来不想用异常的，但是考虑到这些数据都是外部提供的，所以还是需要异常处理，防止程序挂掉
    try
    {
        json jRecv = json::parse(rxData.GetData());

        for (auto &methodFunc : methodFunc_)
        {
            const auto &nameSpace = methodFunc.first; // 第一个是命名空间名字
            auto &funcMap = methodFunc.second;        // 第二个是method子map

            if (jRecv.find(nameSpace) == jRecv.end())
            {
                continue;
            }

            const string &method = jRecv.at(nameSpace).at("method");
            json params; // 可以不存在params
            if (jRecv.at(nameSpace).contains("params") == true)
            {
                params = jRecv.at(nameSpace).at("params");
            }
            bool noPrint = true;
            for (const auto &filter : traceFilter_)
            {
                if (filter == method)
                {
                    noPrint = false;
                }
            }
            if (noPrint)
            {
                // LOG_TRACE_SDK("rpc req {}", nlohmann::to_string(jRecv) + '\0');
            }

            if (funcMap.find(method) != funcMap.end())
            {
                json result = funcMap[method](params);
                // 无返回值的情况
                if (result.empty())
                {
                    continue;
                }
                // 构造一个json类，转成字符串并发出去
                json jSend;
                jSend[nameSpace]["method"] = method;
                jSend[nameSpace]["result"] = result;
                if (noPrint)
                {
                    //  LOG_TRACE_SDK("rpc resp {}", nlohmann::to_string(jSend) + '\0');
                }
                string tx = nlohmann::to_string(jSend) + '\0';
                ProtocolConstruct txData((uint8_t *)tx.c_str(), tx.size());
                txData.SetOption(0, 0);                   // 字符串为自定义字符串
                txData.SetSequence(rxData.GetSequence()); // 复制接收的sequence

                GetMiniServer().Send(info, txData);
            }
        }
    }
    catch (std::exception &)
    {
        //   LOG_WARN_SDK("rpc call exception");
    }
}

/**
 * @description: json发送
 * @param info
 * @param method
 * @return {}
 */
void SdkProtocolServer::JsonSend(const NetClientInfo &info, const nlohmann::json &jSend)
{
    ProtocolConstruct txMsg(nlohmann::to_string(jSend) + '\0');
    GetMiniServer().Send(info, txMsg);
}

/**
 * @description: 添加RPC回调
 * @param &nameSpace
 * @param &method
 * @param func
 * @return {}
 */
void SdkProtocolServer::AddMethodCall(const std::string &nameSpace, const std::string &method, RpcServerCallbackType func)
{
    methodFunc_[nameSpace][method] = func;
}

/**
 * @description: 添加二进制字段的回调
 * @param method
 * @param func
 * @return {}
 */
void SdkProtocolServer::AddBinCall(const std::string &method, BinCallbackType func) { binFunc_[method] = func; }

/**
 * @description: 协议是二进制数据，进行解析
 * @param &info 客户端信息，二进制是通过TCP的
 * @param &rxData
 * @return {}
 */
void SdkProtocolServer::BinRecv(const NetClientInfo &info, const ProtocolParse &rxData)
{
    BinPackParse binDatas(rxData.GetData());
    for (uint32_t i = 0; i < binDatas.GetSize(); i++)
    {
        string method = binDatas.GetTag(i);
        if (binFunc_.count(method))
        {
            auto ret = binFunc_[method](info, binDatas.GetData(i));
            // 若有返回值，则组包进行发送，否则就是不回复
            // 只支持发一个包
            if (ret.empty() == false)
            {
                BinPackConstruct binPack(method, ret);
                ProtocolConstruct txData(reinterpret_cast<const uint8_t *>(&binPack.GetData()), binPack.GetSize());
                GetMiniServer().Send(info, txData);
            }
        }
    }
}

bool SdkProtocolServer::BinSend(const NetClientInfo &info, const std::vector<BinPackConstruct> &data)
{
    ProtocolConstruct txData(reinterpret_cast<const uint8_t *>(data.data()), data.size() * sizeof(BinPackConstruct));
    txData.SetOption(1, 2);
    return GetMiniServer().Send(info, txData);
}

bool SdkProtocolServer::BinSend(const NetClientInfo &info, const BinPackConstruct &data)
{
    ProtocolConstruct txData(data.GetData());
    txData.SetOption(1, 2);
    return GetMiniServer().Send(info, txData);
}

/**
 * @description: 获取当前处理的客户端信息，只在recv回调函数中有效，非recv返回空
 * @return {}
 */
NetClientInfo SdkProtocolServer::GetClientInfo() { return curClient_; }

bool SdkProtocolServer::RawSend(const NetClientInfo &info, uint8_t opt, uint16_t extPot, const std::vector<uint8_t> &data)
{
    ProtocolConstruct txData(data);
    txData.SetOption(opt, extPot);
    return GetMiniServer().Send(info, txData);
}

/**
 * @description: 注册订阅，只有注册的订阅函数才能被客户端请求订阅
 * @param &name
 * @param ms 默认的上报间隔，必须大于0，对条件上报的，建议设置为一半
 * @param func
 * @return {}
 */
void SdkProtocolServer::AddSubscribeCall(const std::string &nameSpace, const std::string &method, int ms, std::function<std::pair<bool, nlohmann::json>()> func)
{
    if (ms <= 0)
    {
        //  LOG_WARN_SDK("subscribe interval can't <= 0!");
        return;
    }
    SdkSubscribeFuncType funcStruct;
    funcStruct.func = func;
    funcStruct.ms = ms;
    funcStruct.nameSpace = nameSpace;
    funcStruct.method = method;

    subscribeInfo_[nameSpace][method] = funcStruct;
}

/**
 * @description: 处理sdk客户端传来的订阅请求
 * @param &params
 * @return {}
 */
nlohmann::json SdkProtocolServer::Subscribe(const nlohmann::json &params)
{
    try
    {
        for (const auto &item : params)
        {
            string nameSpace = item.at("namespace");
            string method = item.at("method");
            int ms = 0;
            if (item.contains("interval"))
            {
                ms = item.at("interval");
            }

            // 不存在则直接异常
            auto &funcStruct = subscribeInfo_.at(nameSpace).at(method);

            // 不存在interval或者interval就是0的情况
            if (ms == 0)
            {
                ms = funcStruct.ms;
            }

            // 若当前没有客户端，则添加定时回调
            if (funcStruct.clients.empty())
            {
                std::function<void(void)> sendFunc = [this, nameSpace, method]()
                { this->subscribeInfo_.at(nameSpace).at(method).Send(); };
                GetMiniServer().AddTimeoutCallback(nameSpace + method, ms, sendFunc);
            }

            funcStruct.clients.insert(GetClientInfo());
            funcStruct.addNew = true; // 注册时强制发送一次，不然可能由于状态自开机以来一直未改变，导致一直不上报

            //  LOG_DEBUG_SDK("subscribe, client:{}:{}, namespace:{}, method:{}, interval:{}",
            //         GetClientInfo().clientIp,
            //         GetClientInfo().clientPort,
            //         nameSpace,
            //         method,
            //         ms);
        }
        return ErrState2json(RetState::ok);
    }
    catch (const std::exception &)
    {
        return ErrState2json(RetState::parseErr);
    }
}

/**
 * @description: 处理sdk客户端传来的取消订阅请求
 * @param &params
 * @return {}
 */
nlohmann::json SdkProtocolServer::Unsubscribe(const nlohmann::json &params)
{
    try
    {
        for (const auto &item : params)
        {
            string nameSpace = item.at("namespace");
            string method = item.at("method");

            auto client = GetClientInfo();
            // 不存在则直接异常
            subscribeInfo_.at(nameSpace).at(method).clients.erase(client);
            // LOG_DEBUG_SDK("unsubscribe, client:{}:{}, namespace:{}, method:{}", GetClientInfo().clientIp, GetClientInfo().clientPort, nameSpace, method);
        }
        return ErrState2json(RetState::ok);
        ;
    }
    catch (const std::exception &)
    {
        return ErrState2json(RetState::parseErr);
    }
}

/**
 * @description: 注册超时调用，函数会定期调用
 * @param &name
 * @param ms
 * @param func
 * @return {}
 */
void SdkProtocolServer::AddTimeoutCall(const std::string &name, int ms, EvtTimeoutCallbackType func)
{
    if (GetMiniServer().AddTimeoutCallback(name, ms, func) == false)
    {
        //   LOG_ERROR_SDK("can't add timeout callback:{}, please rename it or remove the old!", name);
    }
}

/**
 * @description: 移除订阅服务器中对应的客户端信息，用于客户端掉线时使用
 * @param &client
 * @return {}
 */
void SdkProtocolServer::RmvSubscribeClient(const NetClientInfo &client)
{
    for (auto &region : subscribeInfo_)
    {
        for (auto &method : region.second)
        {
            method.second.clients.erase(client);
        }
    }
}

/**
 * @description: 判断客户端是否在线
 * 客户端会定期发送AreYouOk，服务端接收后清除超时计数
 * 此函数会删除超过超时计数超过一定值的客户端
 * @return {}
 */
void SdkProtocolServer::CheckClientOnlineStatus()
{
    // 遍历，掉线删除，不掉线则发送
    for (auto it = clients_.begin(); it != clients_.end();)
    {
        if (it->lost > 5)
        {
            //  LOG_DEBUG_SDK("Udp client {}:{} lost", it->clientIp, it->clientPort);
            RmvSubscribeClient(*it);
            it = clients_.erase(it);
        }
        else
        {
            it->lost++; // 增加掉线计数
            ++it;
        }
    }
}

/**
 * @description: 收到心跳包数据
 * @param params
 * @return {}
 */
nlohmann::json SdkProtocolServer::RxAreYouOk(const nlohmann::json &result)
{
    (void)result;
    try
    {
        auto info = GetClientInfo();

        // 新增或旧客户端lost清零
        auto it = std::find(clients_.begin(), clients_.end(), info);
        if (it != clients_.end())
        {
            it->lost = 0;
        }
        else
        {
            info.lost = 0;
            clients_.push_back(info);
        }
    }
    catch (std::exception &)
    {
        return false;
    }

    return true;
}

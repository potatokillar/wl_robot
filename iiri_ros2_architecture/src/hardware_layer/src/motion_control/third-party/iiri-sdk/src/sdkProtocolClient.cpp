/*
 * @Author: 唐文浩-0036
 * @Date: 2023-07-14
 * @LastEditors: 唐文浩
 * @LastEditTime: 2025-06-23
 * @Description:
 *
 * Copyright (c) 2022-2023 by 西湖大学智能产业研究院, All Rights Reserved.
 */
#include "sdkProtocolClient.hpp"

#include <iostream>
using namespace std;
using namespace nlohmann;

namespace iiri
{
    SdkProtocol::SdkProtocol(const std::string &ip)
    {
        udpCli_ = make_shared<UdpClient>(ip, 20333);
        udpCli_->AsyncRecv([this](const vector<uint8_t> &data)
                           { RecvCallback(data); });
        thread_ = thread(&UdpClient::Loop, this->udpCli_);
    }
    SdkProtocol::~SdkProtocol()
    {
        udpCli_->Stop();
        if (thread_.joinable())
        {
            thread_.join();
        }
    }

    /**
     * @description: SDK调用
     * @param txData
     * @return {}
     */
    Result<nlohmann::json> SdkProtocol::Call(const std::string &nameSpace, const std::string &method, const nlohmann::json &params, uint32_t ms)
    {
        // 若阻塞时间太短，置为50
        if (ms < 50)
        {
            ms = 50;
        }
        Result<nlohmann::json> ret(RetState::netErr, false);
        json jSend;
        jSend[nameSpace]["method"] = method;
        jSend[nameSpace]["params"] = params;
        ProtocolConstruct txMsg(nlohmann::to_string(jSend) + '\0'); // json转成字符串后再构造消息

        udpCli_->Send(txMsg.GetData());

        auto seq = txMsg.GetSequence();
        std::unique_lock<std::mutex> lock(mutex_);
        // 遍历收到的值，且收到sequence一致的值认为调用返回
        // 理论上支持多个同时调用，但未测试
        // todo 20ms的超时在本机测试居然会超时，不太合理但原因未知，待进一步研究
        if (cv_.wait_for(lock, std::chrono::milliseconds(ms), [this, seq]
                         {
            for (const auto &v : allRxData_) {
                if (v.GetSequence() == seq) {
                    return true;
                }
            }
            return false; }))
        {
            // 数据已完成
            for (auto it = allRxData_.begin(); it != allRxData_.end();)
            {
                if (it->GetSequence() == seq)
                {
                    // json类型
                    if (it->GetOption() == 0)
                    {
                        try
                        {
                            json jRecv = json::parse(it->GetData());
                            if (jRecv.contains(nameSpace) == false)
                            {
                                ret.first = RetState::parseErr;
                            }

                            jRecv = jRecv.at(nameSpace); // 必须是同一命名空间

                            string rxMethod = jRecv.at("method");
                            json result = jRecv.at("result");

                            if (rxMethod == method)
                            {
                                ret.first = RetState::ok; // 回复正常，但不代表回复值认为正常
                                ret.second = result;
                            }
                        }
                        catch (std::exception &)
                        {
                        }
                    }
                    else if ((it->GetOption() == 1) && (it->GetExtOption() == 2))
                    {
                        // 二进制协议
                    }
                    it = allRxData_.erase(it); // 处理完需删除
                }
                else
                {
                    ++it;
                }
            }
        }
        else
        {
            // 超时
            // std::cout << "Timeout! Data is not ready." << std::endl;
            ret.first = RetState::netErr;
        }
        return ret;
    }

    /**
     * @description: 异步调用，需要额外设置异步调用的返回回调函数
     * @param nameSpace
     * @param method
     * @param params
     * @return {}
     */
    void SdkProtocol::AsyncCall(const std::string &nameSpace, const std::string &method, const nlohmann::json &params)
    {
        json jSend;
        jSend[nameSpace]["method"] = method;
        jSend[nameSpace]["params"] = params;
        // json转成字符串后再构造消息
        asyncTxMsg_ = std::make_unique<ProtocolConstruct>(nlohmann::to_string(jSend) + '\0');
        // 异步调用需要保证真正调用时指向的内存未被释放
        udpCli_->Send(asyncTxMsg_->GetData());
    }

    /**
     * @description: 快速发送，即不判断数据是否成功，服务端数据也不会返回
     * @param nameSpace
     * @param method
     * @param params
     * @return {}
     */
    void SdkProtocol::QuickCall(const std::string &nameSpace, const std::string &method, const nlohmann::json &params)
    {
        json jSend;
        jSend[nameSpace]["method"] = method;
        jSend[nameSpace]["params"] = params;
        ProtocolConstruct txMsg(nlohmann::to_string(jSend) + '\0'); // json转成字符串后再构造消息

        udpCli_->Send(txMsg.GetData());
    }

    /**
     * @description: 异步接收回调函数
     * @return {}
     */
    void SdkProtocol::RecvCallback(const std::vector<uint8_t> &data)
    {
        ProtocolParse rxData(data);
        if (rxData.Empty() == true)
        {
            udpCli_->AsyncRecv([this](const vector<uint8_t> &data)
                              { RecvCallback(data); });
            return;
        }

        // 若是json协议，先看看该协议是被动接收还是发送后等待接收
        // 目前只有心跳协议
        // todo 需要确认下json解析耗时
        if (rxData.GetOption() == 0)
        {
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
                    const json &params = jRecv.at(nameSpace).at("params");
                    if (funcMap.find(method) != funcMap.end())
                    {
                        json result = funcMap[method](params);
                        // 构造一个json类，转成字符串并发出去
                        json jSend;
                        jSend[nameSpace]["method"] = method;
                        jSend[nameSpace]["result"] = result;

                        string tx = nlohmann::to_string(jSend) + '\0';
                        ProtocolConstruct txData((uint8_t *)tx.c_str(), tx.size());
                        txData.SetOption(0, 0);                   // 字符串为自定义字符串
                        txData.SetSequence(rxData.GetSequence()); // 复制接收的sequence
                        udpCli_->Send(txData.GetData());
                        udpCli_->AsyncRecv([this](const vector<uint8_t> &data)
                                          { RecvCallback(data); }); // 开启下一个异步接收
                        return;                                     // 解析成功，退出
                    }
                }

                for (auto &func : jsonParseFuncVoid_)
                {
                    const auto &nameSpace = func.first; // 第一个是命名空间名字
                    auto &funcMap = func.second;        // 第二个是method子map
                    if (jRecv.find(nameSpace) == jRecv.end())
                    {
                        continue;
                    }
                    const string &method = jRecv.at(nameSpace).at("method");
                    const json &params = jRecv.at(nameSpace).at("result");
                    if (funcMap.find(method) != funcMap.end())
                    {
                        funcMap[method](params);
                        udpCli_->AsyncRecv([this](const vector<uint8_t> &data)
                                          { RecvCallback(data); }); // 开启下一个异步接收
                        return;                                     // 解析成功，退出
                    }
                }
            }
            catch (std::exception &)
            {
            }
        }
        else if ((rxData.GetOption() == 1) && (rxData.GetExtOption() == 2))
        {
            // 二进制协议
            BinPackParse binDatas(rxData.GetData());
            for (uint32_t i = 0; i < binDatas.GetSize(); i++)
            {
                string method = binDatas.GetTag(i);
                if (binParseFunc_.count(method))
                {
                    binParseFunc_[method](binDatas.GetData(i));
                }
            }
        }

        std::lock_guard<std::mutex> lock(mutex_);
        allRxData_.push_back(rxData);

        // 通知等待的线程
        cv_.notify_all();
        udpCli_->AsyncRecv([this](const vector<uint8_t> &data)
                          { RecvCallback(data); }); // 开启下一个异步接收
    }

    /**
     * @description: 设置json协议解析，对服务端主动发送的协议进行解析，客户端回复的协议由call进行解析
     * @param nameSpace
     * @param method
     * @param func 带回复的回调
     * @return {}
     */
    void SdkProtocol::SetJsonParse(const std::string &nameSpace, const std::string &method, JsonParseFunc func)
    {
        methodFunc_[nameSpace][method] = func;
    }

    /**
     * @description: 对服务器主动发送的json协议进行解析
     * @param &nameSpace
     * @param &method
     * @param func 无返回只接收的回调
     * @return {}
     */
    void SdkProtocol::SetJsonParse(const std::string &nameSpace, const std::string &method, JsonParseFuncVoid func)
    {
        jsonParseFuncVoid_[nameSpace][method] = func;
    }

    /**
     * @description: 对服务器主动发送的二进制协议进行解析
     * @param &tag
     * @param func
     * @return {}
     */
    void SdkProtocol::SetBinParse(const std::string &tag, BinParseFuncVoid func) { binParseFunc_[tag] = func; }

    /**
     * @description: 二进制发送，虽然协议支持多个级联，但是API目前只支持一个
     * @param &tag
     * @param &bodyData
     * @return {}
     */
    bool SdkProtocol::BinCall(const std::string &tag, const std::vector<uint8_t> &bodyData)
    {
        BinPackConstruct binPack(tag, bodyData);
        return udpCli_->Send(binPack.GetData());
    }

    /**
     * @description: 设置订阅，客户端断线重连需要重新订阅
     * @param nameSpace
     * @param method
     * @param interval
     * @return {}
     */
    RetState SdkProtocol::SetSubscribe(const std::string &nameSpace, const std::string &method, std::function<void(nlohmann::json)> func,
                                       uint32_t interval)
    {
        nlohmann::json param;
        nlohmann::json item;
        item["namespace"] = nameSpace;
        item["method"] = method;
        if (interval != 0)
        {
            item["interval"] = interval;
        }
        param.push_back(item);
        Call("base", "Subscribe", param);

        SetJsonParse(nameSpace, method, func);
        return RetState::ok;
    }

    /**
     * @description: 取消订阅，若客户端掉线，服务器也会自动取消订阅
     * @param nameSpace
     * @param method
     * @return {}
     */
    void SdkProtocol::SetUnsubscribe(const std::string &nameSpace, const std::string &method)
    {
        nlohmann::json param;
        nlohmann::json item;
        item["namespace"] = nameSpace;
        item["method"] = method;
        param.push_back(item);
        Call("base", "Unsubscribe", param);
    }

    static const std::map<string, RetState> STRING_RETSTATE = {
        {"ok", RetState::ok},
        {"netErr", RetState::netErr},
        {"outRange", RetState::outRange},
        {"timeout", RetState::timeout},
        {"noSupport", RetState::noSupport},
        {"parseErr", RetState::parseErr},
        {"noUpdate", RetState::noUpdate},
        {"busy", RetState::busy},
        {"interrupt", RetState::interrupt},
        {"noExist", RetState::noExist},
        {"error", RetState::error},
    };

    RetState Json2errState(const nlohmann::json &in)
    {
        if (STRING_RETSTATE.count(in))
        {
            return STRING_RETSTATE.at(in);
        }
        return RetState::error;
    }

    void SdkProtocol::SetEventCallback(std::function<void(RobotEvent)> func)
    {
        // UDP client doesn't have event callbacks, just store the callback for potential future use
        (void)func;
    }

} // namespace iiri

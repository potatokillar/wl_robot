/*
 * @Author: 唐文浩-0036
 * @Date: 2023-07-14
 * @LastEditors: 唐文浩
 * @LastEditTime: 2025-06-23
 * @Description: 智研院SDK协议的解包和封包
 *
 * Copyright (c) 2022-2023 by 西湖大学智能产业研究院, All Rights Reserved.
 */
#pragma once
#include <boost/asio.hpp>
#include <condition_variable>
#include <iomanip>
#include <list>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "nlohmann/json.hpp"
#include "protocol.hpp"
#include "type/commType.hpp"
#include "udpClient.hpp"
// #include "websocket.hpp"

namespace iiri
{
    using JsonParseFuncVoid = std::function<void(const nlohmann::json &)>;       // json解析，无回复
    using JsonParseFunc = std::function<nlohmann::json(const nlohmann::json &)>; // json解析，有回复
    using BinParseFuncVoid = std::function<void(const std::vector<uint8_t> &)>;  // 二进制解析，无回复

    class SdkProtocol
    {
    public:
        SdkProtocol(const std::string &ip);
        ~SdkProtocol();
        Result<nlohmann::json> Call(const std::string &nameSpace, const std::string &method, const nlohmann::json &params, uint32_t ms = 50);
        void AsyncCall(const std::string &nameSpace, const std::string &method, const nlohmann::json &params);
        void QuickCall(const std::string &nameSpace, const std::string &method, const nlohmann::json &params);
        void SetJsonParse(const std::string &nameSpace, const std::string &method, JsonParseFunc func);
        void SetJsonParse(const std::string &nameSpace, const std::string &method, JsonParseFuncVoid func);
        void SetBinParse(const std::string &tag, BinParseFuncVoid func);

        bool BinCall(const std::string &tag, const std::vector<uint8_t> &bodyData);
        void SetDelayFunc(uint32_t ms, DelayFunc func) { udpCli_->SetDelayFunc(ms, func); }

        RetState SetSubscribe(const std::string &nameSpace, const std::string &method, std::function<void(nlohmann::json)> func,
                              uint32_t interval = 0);
        void SetUnsubscribe(const std::string &nameSpace, const std::string &method);

        void SetEventCallback(std::function<void(RobotEvent)> func);

    private:
        void RecvCallback(const std::vector<uint8_t> &rxData);

        std::shared_ptr<UdpClient> udpCli_;
        std::list<ProtocolParse> allRxData_;
        std::mutex mutex_;
        std::condition_variable cv_;
        bool rxReady_ = false;
        std::map<std::string, std::map<std::string, JsonParseFunc>> methodFunc_;
        std::map<std::string, std::map<std::string, JsonParseFuncVoid>> jsonParseFuncVoid_;
        std::map<std::string, BinParseFuncVoid> binParseFunc_;

        std::thread thread_;
        std::unique_ptr<ProtocolConstruct> asyncTxMsg_;
    };

    RetState Json2errState(const nlohmann::json &in);

    inline std::string Double2Json(double number, size_t width = 6)
    {
        std::stringstream ss;
        ss << std::setprecision(width) << number;
        return ss.str();
    }

    inline double Json2Double(nlohmann::json data) { return stod(std::string(data)); }

} // namespace iiri

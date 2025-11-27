/*
 * @Author: 唐文浩
 * @Date: 2022-08-10
 * @LastEditors: 唐文浩
 * @LastEditTime: 2025-02-28
 * @Description:
 *
 * Copyright (c) 2022-2022 by 西湖大学智能产业研究院, All Rights Reserved.
 */
#include "debugWatch.hpp"

#include <iostream>

#include "apiDebug.hpp"
#include "apiDevice.hpp"

using namespace std;

namespace iiri::debug
{

    DebugWatch::DebugWatch(const std::string &ip)
    {
        rpcCli_ = make_shared<SdkProtocol>(ip);
        apiDbg_ = make_unique<ApiDebug>(rpcCli_);
        apiDev_ = make_unique<ApiDevice>(rpcCli_);
        SubscribeDebug();
    }

    // 析构和移动使用默认
    DebugWatch::~DebugWatch() = default;
    DebugWatch::DebugWatch(DebugWatch &&rhs) = default;
    DebugWatch &DebugWatch::operator=(DebugWatch &&rhs) = default;

    // unique_ptr不支持拷贝，需要特殊定义
    DebugWatch::DebugWatch(const DebugWatch &rhs)
    {
        apiDbg_ = make_unique<ApiDebug>(*rhs.apiDbg_);
        rpcCli_ = rhs.rpcCli_; // shared_ptr支持拷贝操作
    }
    DebugWatch &DebugWatch::operator=(const DebugWatch &rhs)
    {
        *apiDbg_ = *rhs.apiDbg_; // 必须是值赋值，因为unique_ptr不支持复制
        rpcCli_ = rhs.rpcCli_;   // 可以直接指针赋值，因为shared_ptr原生支持复制
        return *this;
    }

    RetState DebugWatch::GetCustomParamInfo(std::vector<ParamInfo> &result) { return apiDbg_->GetCustomParamInfo(result); }

    RetState DebugWatch::SetCustomParam(const std::vector<ParamValue> &set) { return apiDbg_->SetCustomParam(set); }

    RetState DebugWatch::GetCustomParam(const std::vector<std::string> &names, std::vector<ParamValue> &result)
    {
        return apiDbg_->GetCustomParam(names, result);
    }

    /**
     * @description: 订阅调试数据上报
     * @return {}
     */
    RetState DebugWatch::SubscribeDebug()
    {
        nlohmann::json param;
        nlohmann::json item;
        item["name"] = "debug";
        param.push_back(item);

        auto ret = rpcCli_->Call("debug", "Subscribe", param);

        // debug数据同时包含以下的字段
        rpcCli_->SetBinParse("leg", [this](const vector<uint8_t> &data)
                             { this->DebugCallbackLegData(data); });
        rpcCli_->SetBinParse("rpy", [this](const vector<uint8_t> &data)
                             { this->DebugCallbackRpyData(data); });
        rpcCli_->SetBinParse("watch", [this](const vector<uint8_t> &data)
                             { this->DebugCallbackWatchData(data); });
        rpcCli_->SetBinParse("arm", [this](const vector<uint8_t> &data)
                             { this->DebugCallbackArmData(data); });
        rpcCli_->SetBinParse("armTeach", [this](const vector<uint8_t> &data)
                             { this->DebugCallbackArmTeachData(data); });
        rpcCli_->SetBinParse("humanInfo", [this](const vector<uint8_t> &data)
                             { this->DebugCallbackHumanInfoData(data); });

        return ret.first;
    }

    void DebugWatch::DebugCallbackLegData(const std::vector<uint8_t> &data)
    {
        if (legFunc_)
        {
            legFunc_(Vector2Struct<LegData>(data));
        }
    }
    void DebugWatch::DebugCallbackRpyData(const std::vector<uint8_t> &data)
    {
        if (rpyFunc_)
        {
            rpyFunc_(Vector2Struct<RpyData>(data));
        }
    }
    void DebugWatch::DebugCallbackWatchData(const std::vector<uint8_t> &data)
    {
        if (watchFunc_)
        {
            watchFunc_(Vector2Struct<WatchData>(data));
        }
    }
    void DebugWatch::DebugCallbackArmData(const std::vector<uint8_t> &data)
    {
        if (armFunc_)
        {
            armFunc_(Vector2Struct<ArmData>(data));
        }
    }

    void DebugWatch::DebugCallbackArmTeachData(const std::vector<uint8_t> &data)
    {
        if (armTeachsFunc_)
        {
            armTeachsFunc_(Vector2Struct<ArmTeachs>(data));
        }
    }

    void DebugWatch::DebugCallbackHumanInfoData(const std::vector<uint8_t> &data)
    {
        /*DEBUG*/ static std::chrono::steady_clock::time_point last_print_time = std::chrono::steady_clock::now();
        /*DEBUG*/ static const auto print_interval = std::chrono::seconds(1); // 每秒打印一次

        if (humanInfoFunc_)
        {
            humanInfoFunc_(Vector2Struct<HumanData>(data));

            // 调试用，检查是否需要打印消息
            auto current_time = std::chrono::steady_clock::now();
            if (current_time - last_print_time >= print_interval)
            {
                std::cout << "humanInfoFunc_(Vector2Struct<msg::net::human::HumanDebug>(data))" << std::endl;
                last_print_time = current_time; // 更新上次打印时间
            }
        }
    }

    bool DebugWatch::IsServerAlive() { return apiDev_->IsServerAlive(); }
} // namespace iiri::debug
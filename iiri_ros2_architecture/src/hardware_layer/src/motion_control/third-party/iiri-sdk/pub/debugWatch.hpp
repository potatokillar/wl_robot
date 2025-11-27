/*
 * @Author: 唐文浩
 * @Date: 2022-07-28
 * @LastEditors: 唐文浩
 * @LastEditTime: 2025-02-28
 * @Description: debug类，仅在debug版本有效，产品中请勿使用
 *
 * Copyright (c) 2022-2022 by 西湖大学智能产业研究院, All Rights Reserved.
 */
#pragma once
#include <memory>

#include "type/debugType.hpp"

namespace iiri
{
    class SdkProtocol;
    class ApiDevice;

    namespace debug
    {
        class ApiDebug;
        using LegCallback = std::function<void(const LegData &)>;
        using RpyCallback = std::function<void(const RpyData &)>;
        using WatchCallback = std::function<void(const WatchData &)>;
        using ArmCallback = std::function<void(const ArmData &)>;
        using ArmTeachsCallback = std::function<void(const ArmTeachs &)>;
        using HumanInfoCallback = std::function<void(const HumanData &)>;

        class DebugWatch
        {
        public:
            DebugWatch(const std::string &ip);
            virtual ~DebugWatch();
            DebugWatch(const DebugWatch &rhs);
            DebugWatch &operator=(const DebugWatch &rhs);
            DebugWatch(DebugWatch &&rhs);
            DebugWatch &operator=(DebugWatch &&rhs);

            RetState SetCustomParam(const std::vector<ParamValue> &set);
            RetState GetCustomParam(const std::vector<std::string> &names, std::vector<ParamValue> &result);
            RetState GetCustomParamInfo(std::vector<ParamInfo> &result);

            void SetCallback(LegCallback func) { legFunc_ = func; }
            void SetCallback(RpyCallback func) { rpyFunc_ = func; }
            void SetCallback(WatchCallback func) { watchFunc_ = func; }
            void SetCallback(ArmCallback func) { armFunc_ = func; }
            void SetCallback(ArmTeachsCallback func) { armTeachsFunc_ = func; }
            void SetCallback(HumanInfoCallback func) { humanInfoFunc_ = func; }

            bool IsServerAlive();

        private:
            RetState SubscribeDebug();

            void DebugCallbackLegData(const std::vector<uint8_t> &data);
            void DebugCallbackRpyData(const std::vector<uint8_t> &data);
            void DebugCallbackWatchData(const std::vector<uint8_t> &data);
            void DebugCallbackArmData(const std::vector<uint8_t> &data);
            void DebugCallbackArmTeachData(const std::vector<uint8_t> &data);
            void DebugCallbackHumanInfoData(const std::vector<uint8_t> &data);

        private:
            std::shared_ptr<SdkProtocol> rpcCli_;
            std::unique_ptr<ApiDebug> apiDbg_;
            std::unique_ptr<ApiDevice> apiDev_;

            LegCallback legFunc_;
            RpyCallback rpyFunc_;
            WatchCallback watchFunc_;
            ArmCallback armFunc_;
            ArmTeachsCallback armTeachsFunc_;
            HumanInfoCallback humanInfoFunc_;
        };
    } // namespace debug
} // namespace iiri

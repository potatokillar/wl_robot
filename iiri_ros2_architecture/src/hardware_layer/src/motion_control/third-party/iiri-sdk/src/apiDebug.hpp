/*
 * @Author: 唐文浩
 * @Date: 2022-08-09
 * @LastEditors: 唐文浩
 * @LastEditTime: 2025-02-28
 * @Description: debug空间的API
 *
 * Copyright (c) 2022-2022 by 西湖大学智能产业研究院, All Rights Reserved.
 */
#pragma once

#include "sdkProtocolClient.hpp"
#include "type/debugType.hpp"

namespace iiri::debug
{
    class ApiDebug
    {
    public:
        ApiDebug(std::shared_ptr<SdkProtocol> rpcCli);
        RetState SetCustomParam(const std::vector<ParamValue> &set);
        RetState GetCustomParam(const std::vector<std::string> &set, std::vector<ParamValue> &result);
        RetState GetCustomParamInfo(std::vector<ParamInfo> &result);

    private:
        Result<nlohmann::json> Call(const std::string &method, const nlohmann::json &params);
        std::shared_ptr<SdkProtocol> rpcCli_;
    };
} // namespace iiri::debug

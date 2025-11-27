/*
 * @Author: 唐文浩
 * @Date: 2023-11-15
 * @LastEditors: 唐文浩
 * @LastEditTime: 2025-02-19
 * @Description: 最基础的sdk功能
 *
 * Copyright (c) 2022-2023 by 西湖大学智能产业研究院, All Rights Reserved.
 */
#pragma once
#include <memory>

#include "sdkProtocolServer.hpp"

class SdkDevice
{
public:
    SdkDevice(std::shared_ptr<SdkProtocolServer> sdk);

private:
    void RegisterSdkCall();
    // std::shared_ptr<ApiDevice> api_;
    std::shared_ptr<SdkProtocolServer> sdk_;

private:
    nlohmann::json FindMe(const nlohmann::json &params);
};
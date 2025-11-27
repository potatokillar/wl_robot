/*
 * @Author: 唐文浩
 * @Date: 2025-02-18
 * @LastEditors: 唐文浩
 * @LastEditTime: 2025-02-19
 * @Description:
 *
 * Copyright (c) 2022-2025 by 西湖大学智能产业研究院, All Rights Reserved.
 */
#include "sdkDevice.hpp"

#include "sdkProtocolServer.hpp"

using namespace nlohmann;
using namespace std;

/**
 * @description: 能用构造函数初始化的，就不要用Init再去初始化
 * @param api
 * @param sdk sdk服务器实例，不保存只用一次
 * @return {}
 */
SdkDevice::SdkDevice(std::shared_ptr<SdkProtocolServer> sdk) : sdk_(sdk) { RegisterSdkCall(); }

/**
 * @description: 向sdk服务器注册回调函数
 * @param sdk服务器
 * @return {}
 */
void SdkDevice::RegisterSdkCall()
{
    // 因心跳包的处理较为特殊，所以将其移到下层sdkProtocolServer中了
    sdk_->AddMethodCall("device", "FindDevice", [this](json params) -> json
                        { return this->FindMe(params); });
}

/**
 * @description: 设备发现回复
 * @param params
 * @return {}
 */
nlohmann::json SdkDevice::FindMe(const nlohmann::json &params)
{
    (void)params;
    const char *domain_id = getenv("ROS_DOMAIN_ID");

    json jRet;
    jRet["name"] = "wl_ros";
    jRet["model"] = "id: " + std::string(domain_id ? domain_id : "0");
    return jRet;
}

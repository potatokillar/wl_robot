/*
 * @Author: 唐文浩
 * @Date: 2025-07-01
 * @LastEditors: 唐文浩
 * @LastEditTime: 2025-07-01
 * @Description:
 *
 * Copyright (c) 2022-2025 by 西湖大学智能产业研究院, All Rights Reserved.
 */
#include "device.hpp"

#include <iostream>

#include "apiDevice.hpp"
#include "metaData.hpp"
#include "sdkProtocolClient.hpp"

using namespace std;
namespace iiri
{
    Device::Device(const std::string &ip)
    {
        rpcCli_ = make_shared<SdkProtocol>(ip);
        apiDev_ = make_unique<ApiDevice>(rpcCli_);
        cout << meta() << endl; // 打印元数据
    }

    // 析构和移动使用默认
    Device::~Device() {}
    Device::Device(Device &&rhs) = default;
    Device &Device::operator=(Device &&rhs) = default;

    /**
     * @description: 元数据
     * @return {}
     */
    const std::string_view &Device::meta() { return SDK_VERSION; }

    /**
     * @description: 获取电池信息，例如电量，状态等
     * @param &info
     * @return {}
     */
    std::pair<RetState, BatteryInfo> Device::GetBatteryInfo()
    {
        std::pair<RetState, BatteryInfo> ret;
        ret.first = apiDev_->GetBatteryInfo(ret.second);
        return ret;
    }
    std::pair<RetState, DeviceInfo> Device::GetDeviceInfo()
    {
        std::pair<RetState, DeviceInfo> ret;
        ret.first = apiDev_->GetDeviceInfo(ret.second);
        return ret;
    }

    RetState Device::SubscribeBatteryInfo(std::function<void(const BatteryInfo &)> func) { return apiDev_->SubscribeBatteryInfo(func); }
    RetState Device::SubscribeGamepadCmd(std::function<void(const GamepadCmd &)> func) { return apiDev_->SubscribeGamepadCmd(func); }

    void Device::SubscribeEvent(std::function<void(RobotEvent evt)> func) { apiDev_->SubscribeRobotEvent(func); }

} // namespace iiri

/*
 * @Author: 唐文浩
 * @Date: 2025-07-01
 * @LastEditors: 唐文浩
 * @LastEditTime: 2025-07-01
 * @Description: 设备侧API，和具体机器人类型无关的API。其他具体机器人头文件中也会包含同样的一份API
 *
 * Copyright (c) 2022-2025 by 西湖大学智能产业研究院, All Rights Reserved.
 */
#pragma once

#include <memory>

#include "type/qrType.hpp"

namespace iiri
{
    class ApiDevice;
    class SdkProtocol;

    class Device
    {
    public:
        Device(const std::string &ip);
        virtual ~Device();
        Device(const Device &rhs) = delete;
        Device &operator=(const Device &rhs) = delete;
        Device(Device &&rhs);
        Device &operator=(Device &&rhs);

        static const std::string_view &meta();

        void SubscribeEvent(std::function<void(RobotEvent evt)> func);

    public:
        /**
         * @description: 获取设备信息
         * @param &info
         * @return {}
         */
        Result<DeviceInfo> GetDeviceInfo();

        /**
         * @description: 获取电池信息，例如电量，状态等
         * @param &info
         * @return {}
         */
        Result<BatteryInfo> GetBatteryInfo();

        /**
         * @description: 订阅电池信息，该值默认以默认间隔或状态改变上报，具体逻辑同实际机器人有关
         * 通常，是定时上报；但出现若充电状态改变等状态，会立即上报。
         * @param func
         * @return {}
         */
        RetState SubscribeBatteryInfo(std::function<void(const BatteryInfo &)> func);

        /**
         * @description: 订阅手柄数据上报
         * @param func
         * @return {}
         */
        RetState SubscribeGamepadCmd(std::function<void(const GamepadCmd &)> func);

    private:
        std::shared_ptr<SdkProtocol> rpcCli_;
        std::unique_ptr<ApiDevice> apiDev_;
    };

} // namespace iiri
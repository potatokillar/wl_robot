/*
 * @Author: 唐文浩
 * @Date: 2022-08-09
 * @LastEditors: 唐文浩
 * @LastEditTime: 2025-07-15
 * @Description: device命名空间的API
 * 必须包含，否则心跳功能将不可用
 * Copyright (c) 2022-2022 by 西湖大学智能产业研究院, All Rights Reserved.
 */
#pragma once

#include "sdkProtocolClient.hpp"
#include "type/commType.hpp"

namespace iiri
{

    class ApiDevice
    {
    public:
        ApiDevice(std::shared_ptr<SdkProtocol> rpcCli);
        RetState GetDeviceInfo(DeviceInfo &info);
        RetState GetBatteryInfo(BatteryInfo &info);
        bool IsServerAlive() { return lost > 5 ? false : true; }

        RetState SubscribeBatteryInfo(std::function<void(const BatteryInfo &)> func);
        RetState SubscribeGamepadCmd(std::function<void(const GamepadCmd &)> func);

        void SubscribeRobotEvent(std::function<void(RobotEvent)> func);

        RetState SetRgbLedToSolidColor(uint8_t red, uint8_t green, uint8_t blue);

        RetState SetRgbLedToBreathing(uint8_t redStart, uint8_t greenStart, uint8_t blueStart, uint8_t redEnd, uint8_t greenEnd,
                                      uint8_t blueEnd, uint8_t period);

        RetState SetRgbLedToAutoBreathing(uint8_t autoCycle, uint8_t autoBrightnessPercentage);

    private:
        Result<nlohmann::json> Call(const std::string &method, const nlohmann::json &params);
        std::shared_ptr<SdkProtocol> rpcCli_;
        void AreYouOk();
        void RxAreYouOk(const nlohmann::json &rx);

        int lost = 0;

        std::function<void(const BatteryInfo &)> batteryInfoFunc_;
        std::function<void(const GamepadCmd &)> gamepadCmdFunc_;

        bool CheckSdkVersion(const std::string &devVersion, const std::string &myVersion);

    private:
        void BatteryInfoCallback(const nlohmann::json &ret);
        void GamepadCallback(const nlohmann::json &ret);
    };
} // namespace iiri
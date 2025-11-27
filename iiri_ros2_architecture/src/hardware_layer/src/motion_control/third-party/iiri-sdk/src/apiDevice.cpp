/*
 * @Author: 唐文浩
 * @Date: 2022-08-09
 * @LastEditors: 唐文浩
 * @LastEditTime: 2025-07-16
 * @Description: device命名空间的API
 *
 * Copyright (c) 2022-2022 by 西湖大学智能产业研究院, All Rights Reserved.
 */
#include "apiDevice.hpp"

#include <iostream>
#include <regex>

#include "metaData.hpp"

using namespace std;

namespace iiri
{
    ApiDevice::ApiDevice(std::shared_ptr<SdkProtocol> rpcCli)
    {
        rpcCli_ = rpcCli;
        rpcCli_->SetDelayFunc(100, [this](const boost::system::error_code &e)
                              { this->AreYouOk(); }); // 设置定时发送函数
        rpcCli_->SetJsonParse("device", "AreYouOk", [this](const nlohmann::json &rx)
                              { this->RxAreYouOk(rx); }); // 设置areyouok的回复解析函数

        // 必须大于设备的SDK版本
        DeviceInfo info;
        auto ret = GetDeviceInfo(info);
        if (ret == RetState::ok)
        {
            if (CheckSdkVersion(info.version.sdk, std::string(SDK_VERSION)) == false)
            {
                std::cerr << "ERROR! this sdk version < device sdk version" << std::endl;
            }
        }
    }
    Result<nlohmann::json> ApiDevice::Call(const std::string &method, const nlohmann::json &params)
    {
        return rpcCli_->Call("device", method, params);
    }

    RetState ApiDevice::GetDeviceInfo(DeviceInfo &info)
    {
        nlohmann::json param;

        auto ret = Call("GetDeviceInfo", param);
        if (ret.first == RetState::ok)
        {
            try
            {
                info.name = ret.second["name"];

                info.version.software = ret.second["version"]["software"];
                info.version.sdk = ret.second["version"]["sdk"];
                info.version.hardware = ret.second["version"]["hardware"];
                info.version.mechanical = ret.second["version"]["mechanical"];

                info.serialNo = ret.second["serialNo"];
                info.model = ret.second["model"];
            }
            catch (const std::exception &e)
            {
                return RetState::parseErr;
            }
        }
        return ret.first;
    }

    RetState ApiDevice::GetBatteryInfo(BatteryInfo &info)
    {
        nlohmann::json param;

        auto ret = Call("GetBatteryInfo", param);
        if (ret.first == RetState::ok)
        {
            try
            {
                info.quantity = Json2Double(ret.second["quantity"]);
                info.charge = ret.second["charge"];
                info.voltage = Json2Double(ret.second["voltage"]);
                info.current = Json2Double(ret.second["current"]);
            }
            catch (const std::exception &e)
            {
                return RetState::parseErr;
            }
        }
        return ret.first;
    }

    void ApiDevice::AreYouOk()
    {
        if (lost < 10)
        {
            lost++;
        }
        rpcCli_->AsyncCall("device", "AreYouOk", "");
        rpcCli_->SetDelayFunc(100, [this](const boost::system::error_code &e)
                              { this->AreYouOk(); }); // 设置下一次的发送
    }
    void ApiDevice::RxAreYouOk(const nlohmann::json &rx)
    {
        (void)rx;
        lost = 0;
    }

    RetState ApiDevice::SubscribeBatteryInfo(std::function<void(const BatteryInfo &)> func)
    {
        batteryInfoFunc_ = func;
        return rpcCli_->SetSubscribe("device", "PubBatteryInfo", [this](const nlohmann::json &ret)
                                     { this->BatteryInfoCallback(ret); });
    }

    RetState ApiDevice::SubscribeGamepadCmd(std::function<void(const GamepadCmd &)> func)
    {
        gamepadCmdFunc_ = func;
        return rpcCli_->SetSubscribe("device", "PubGamepadCmd", [this](const nlohmann::json &ret)
                                     { this->GamepadCallback(ret); });
    }

    void ApiDevice::BatteryInfoCallback(const nlohmann::json &ret)
    {
        BatteryInfo info;
        try
        {
            info.quantity = Json2Double(ret.at("quantity"));
            info.charge = ret.at("charge");
            info.voltage = Json2Double(ret.at("voltage"));
            info.current = Json2Double(ret.at("current"));
        }
        catch (const std::exception &e)
        {
            return;
        }

        if (batteryInfoFunc_)
        {
            batteryInfoFunc_(info);
        }
    }

    void ApiDevice::GamepadCallback(const nlohmann::json &ret)
    {
        GamepadCmd cmd;
        try
        {
            cmd.a = ret.at("a");
            cmd.b = ret.at("b");
            cmd.back = ret.at("back");
            cmd.down = ret.at("down");
            cmd.lb = ret.at("lb");
            cmd.left = ret.at("left");
            cmd.lp = ret.at("lp");
            cmd.rb = ret.at("rb");
            cmd.right = ret.at("right");
            cmd.rp = ret.at("rp");
            cmd.start = ret.at("start");
            cmd.up = ret.at("up");
            cmd.x = ret.at("x");
            cmd.y = ret.at("y");

            cmd.lx = Json2Double(ret.at("lx"));
            cmd.ly = Json2Double(ret.at("ly"));
            cmd.lt = Json2Double(ret.at("lt"));
            cmd.rt = Json2Double(ret.at("rt"));
            cmd.rx = Json2Double(ret.at("rx"));
            cmd.ry = Json2Double(ret.at("ry"));

            cmd.takeover = ret.at("takeover");
        }
        catch (const std::exception &e)
        {
            std::cout << "Parse gamepad cmd error:" << e.what() << std::endl;
            return;
        }

        if (gamepadCmdFunc_)
        {
            gamepadCmdFunc_(cmd);
        }
    }

    void ApiDevice::SubscribeRobotEvent(std::function<void(RobotEvent)> func) { rpcCli_->SetEventCallback(func); }

    RetState ApiDevice::SetRgbLedToSolidColor(uint8_t red, uint8_t green, uint8_t blue)
    {
        nlohmann::json param;
        param["red"] = red;
        param["green"] = green;
        param["blue"] = blue;

        auto ret = Call("SetRgbLedToSolidColor", param);
        if (ret.first == RetState::ok)
        {
            return RetState::ok;
        }
        else
        {
            return ret.first;
        }
    }

    RetState ApiDevice::SetRgbLedToBreathing(uint8_t redStart, uint8_t greenStart, uint8_t blueStart, uint8_t redEnd, uint8_t greenEnd,
                                             uint8_t blueEnd, uint8_t period)
    {
        nlohmann::json param;
        param["red_start"] = redStart;
        param["green_start"] = greenStart;
        param["blue_start"] = blueStart;
        param["red_end"] = redEnd;
        param["green_end"] = greenEnd;
        param["blue_end"] = blueEnd;
        param["period"] = period;

        auto ret = Call("SetRgbLedToBreathing", param);
        if (ret.first == RetState::ok)
        {
            return RetState::ok;
        }
        else
        {
            return ret.first;
        }
    }

    RetState ApiDevice::SetRgbLedToAutoBreathing(uint8_t autoCycle, uint8_t autoBrightnessPercentage)
    {
        nlohmann::json param;
        param["period"] = autoCycle;
        param["brightness_percentage"] = autoBrightnessPercentage;

        auto ret = Call("SetRgbLedToAutoBreathing", param);
        if (ret.first == RetState::ok)
        {
            return RetState::ok;
        }
        else
        {
            return ret.first;
        }
    }

    /**
     * @description: 检查SDK版本，需要SDK版本大于等于设备中的SDK版本
     * @param devVersion
     * @param myVersion
     * @return {}
     */
    bool ApiDevice::CheckSdkVersion(const std::string &devVersion, const std::string &myVersion)
    {
        std::regex versionRegex(R"(v(\d+)\.(\d+)\.(\d+))");
        std::smatch match;

        int devMajor = 0, devMinor = 0, devPatch = 0;
        int myMajor = 0, myMinor = 0, myPatch = 0;

        if (std::regex_search(devVersion, match, versionRegex) && match.size() == 4)
        {
            devMajor = std::stoi(match[1].str());
            devMinor = std::stoi(match[2].str());
            devPatch = std::stoi(match[3].str());
        }
        else
        {
            return false;
        }

        if (std::regex_search(myVersion, match, versionRegex) && match.size() == 4)
        {
            myMajor = std::stoi(match[1].str());
            myMinor = std::stoi(match[2].str());
            myPatch = std::stoi(match[3].str());
        }
        else
        {
            return false;
        }

        if (myMajor < devMajor)
        {
            return false;
        }

        if (myMinor < devMinor)
        {
            return false;
        }

        if (myPatch < devPatch)
        {
            return false;
        }
        return true;
    }

} // namespace iiri
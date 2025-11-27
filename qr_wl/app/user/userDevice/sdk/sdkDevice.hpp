
#pragma once
#include <memory>

#include "apiDevice.hpp"
#include "canBridge.hpp"
#include "sdkProtocolServer.hpp"

class SdkDevice
{
public:
    SdkDevice(std::shared_ptr<ApiDevice> api, std::shared_ptr<SdkProtocolServer> sdk);

private:
    void RegisterSdkCall();
    std::shared_ptr<ApiDevice> api_;
    std::shared_ptr<SdkProtocolServer> sdk_;

private:
    nlohmann::json GetDeviceInfo(const nlohmann::json& params);
    nlohmann::json GetBatteryInfo(const nlohmann::json& params);
    std::pair<bool, nlohmann::json> PubBatteryInfo();
    nlohmann::json FindMe(const nlohmann::json& params);
};
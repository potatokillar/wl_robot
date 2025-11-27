
#pragma once
#include <memory>

// #include "debugServer.hpp"
#include "deviceDebugServer.hpp"
#include "sdkProtocolServer.hpp"

/**
 * @description: SdkDebug类，负责各业务中产生、发送debug数据的模块。
 * @return {}
 */
class SdkDebug
{
public:
    SdkDebug(std::shared_ptr<SdkProtocolServer> sdk, std::unique_ptr<DeviceDebugServer> ptr_);

private:
    void RegisterSdkCall();

    nlohmann::json SetCustomParam(const nlohmann::json& params);
    nlohmann::json GetCustomParam(const nlohmann::json& params);
    nlohmann::json GetCustomParamInfo(const nlohmann::json& params);
    nlohmann::json GetDebugData(const nlohmann::json& params);
    void PubDebugData();

    std::shared_ptr<SdkProtocolServer> sdk_;
    std::unique_ptr<DeviceDebugServer> ptr_dbg_;
    std::set<NetClientInfo> debugClients_;
};
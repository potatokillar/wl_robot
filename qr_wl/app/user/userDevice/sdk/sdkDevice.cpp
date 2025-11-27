#include "sdkDevice.hpp"

#include "baseline.hpp"
#include "canBridge.hpp"
#include "deviceCustomParam.hpp"
#include "netMsg.hpp"
#include "robotMedia.hpp"
#include "robotState.hpp"
#include "sdkProtocolServer.hpp"

using namespace nlohmann;
using namespace std;

/**
 * @description: 能用构造函数初始化的，就不要用Init再去初始化
 * @param api
 * @param sdk sdk服务器实例，不保存只用一次
 * @return {}
 */
SdkDevice::SdkDevice(std::shared_ptr<ApiDevice> api, std::shared_ptr<SdkProtocolServer> sdk) : api_(api), sdk_(sdk) { RegisterSdkCall(); }

/**
 * @description: 向sdk服务器注册回调函数
 * @param sdk服务器
 * @return {}
 */
void SdkDevice::RegisterSdkCall()
{
    // 因心跳包的处理较为特殊，所以将其移到下层sdkProtocolServer中了
    sdk_->AddMethodCall("device", "GetDeviceInfo", [this](json params) -> json { return this->GetDeviceInfo(params); });
    sdk_->AddMethodCall("device", "GetBatteryInfo", [this](json params) -> json { return this->GetBatteryInfo(params); });
    sdk_->AddMethodCall("device", "FindDevice", [this](json params) -> json { return this->FindMe(params); });

    // 支持订阅的功能
    sdk_->AddSubscribeCall("device", "PubBatteryInfo", 10000, [this]() -> std::pair<bool, json> { return this->PubBatteryInfo(); });
}

json SdkDevice::GetDeviceInfo(const json& params)
{
    (void)params;
    LOG_DEBUG("GetDeviceInfo");
    DeviceInfo info = api_->GetDeviceInfo();

    json ret;
    ret["name"] = info.name;

    ret["version"]["sdk"] = info.version.sdk;
    ret["version"]["software"] = info.version.software;
    ret["version"]["hardware"] = info.version.hardware;
    ret["version"]["mechanical"] = info.version.mechanical;

    ret["serialNo"] = info.serialNo;
    ret["model"] = info.model;
    return ret;
}

/**
 * @description: 主动获取电池信息，未更新则返回旧数据
 * @param params
 * @return {}
 */
json SdkDevice::GetBatteryInfo(const json& params)
{
    (void)params;
    return PubBatteryInfo().second;
}

/**
 * @description: 获取电池数据
 * @return {} true表示数据已更新
 */
std::pair<bool, json> SdkDevice::PubBatteryInfo()
{
    json jRet;

    jRet["current"] = Double2Json(api_->GetBC());
    jRet["quantity"] = Double2Json(api_->GetQuantity());
    jRet["voltage"] = Double2Json(api_->GetSmallBMSVoltage());
    jRet["charge"] = false;  // 目前不支持边冲边用

    //  cout << "t " << api_->dev_.GetQuantity().second << endl;
    bool ret = true;

    return {ret, jRet};
}

/**
 * @description: 设备发现回复
 * @param params
 * @return {}
 */
nlohmann::json SdkDevice::FindMe(const nlohmann::json& params)
{
    (void)params;
    auto info = api_->GetDeviceInfo();

    json jRet;
    jRet["name"] = info.name;
    jRet["model"] = info.model;
    return jRet;
}

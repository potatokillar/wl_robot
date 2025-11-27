
#pragma once
#include <memory>

#include "apiArm.hpp"
#include "sdkProtocolServer.hpp"

class SdkArm
{
public:
    SdkArm(std::shared_ptr<ApiArm> api, std::shared_ptr<SdkProtocolServer> sdk);

private:
    void RegisterSdkCall();

    nlohmann::json SetMoveJTask(const nlohmann::json& params);
    nlohmann::json SetMoveLTask(const nlohmann::json& params);
    nlohmann::json WaitTaskComplete(const nlohmann::json& params);

    nlohmann::json SetCtrlState(const nlohmann::json& params);

    nlohmann::json GetToolNameList(const nlohmann::json& params);
    nlohmann::json SetToolName(const nlohmann::json& params);
    nlohmann::json GetToolName(const nlohmann::json& params);
    nlohmann::json SetToolValue(const nlohmann::json& params);
    nlohmann::json GetToolValue(const nlohmann::json& params);

    nlohmann::json GetNowAngle(const nlohmann::json& params);
    std::pair<bool, nlohmann::json> PubNowAngle();

    nlohmann::json GetNowCart(const nlohmann::json& params);
    std::pair<bool, nlohmann::json> PubNowCart();

    nlohmann::json GetArmState(const nlohmann::json& params);
    std::pair<bool, nlohmann::json> PubArmState();

    nlohmann::json GetFkine6(const nlohmann::json& params);
    nlohmann::json GetIkine6(const nlohmann::json& params);

    nlohmann::json GetFkine7(const nlohmann::json& params);
    nlohmann::json GetIkine7(const nlohmann::json& params);

    nlohmann::json SetIoOutState(const nlohmann::json& params);
    nlohmann::json GetIoOutState(const nlohmann::json& params);
    nlohmann::json GetIoInState(const nlohmann::json& params);
    std::pair<bool, nlohmann::json> PubIoOutState();
    std::pair<bool, nlohmann::json> PubIoInState();

    nlohmann::json SetPositionLimit(const nlohmann::json& params);
    nlohmann::json GetPositionLimit(const nlohmann::json& params);

    nlohmann::json SetCollisionLevel(const nlohmann::json& params);
    nlohmann::json GetCollisionLevel(const nlohmann::json& params);

    nlohmann::json GetArmInfo(const nlohmann::json& params);
    std::pair<bool, nlohmann::json> PubArmInfo();

    nlohmann::json SetJointSpeedLimit(const nlohmann::json& params);
    nlohmann::json GetJointSpeedLimit(const nlohmann::json& params);

    nlohmann::json SetJointAccLimit(const nlohmann::json& params);
    nlohmann::json GetJointAccLimit(const nlohmann::json& params);

    nlohmann::json SetDragMode(const nlohmann::json& params);
    nlohmann::json GetDragMode(const nlohmann::json& params);

    nlohmann::json SetRapidrate(const nlohmann::json& params);
    nlohmann::json GetRapidrate(const nlohmann::json& params);

    nlohmann::json SetReset(const nlohmann::json& params);
    nlohmann::json GetReset(const nlohmann::json& params);
    nlohmann::json SetGripper(const nlohmann::json& params);

private:
    std::shared_ptr<ApiArm> api_;
    std::shared_ptr<SdkProtocolServer> sdk_;

    void Start();
    void Stop();

    arm::ArmState preArmSta_;
    std::map<std::string, std::bitset<32>> preIoOutSta_, preIoInSta_;
};
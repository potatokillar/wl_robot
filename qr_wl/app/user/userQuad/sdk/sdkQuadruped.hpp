
#pragma once
#include <memory>
#include <set>

#include "apiQuadruped.hpp"
#include "sdkProtocolServer.hpp"

using RpcServerCallbackType = std::function<nlohmann::json(nlohmann::json)>;

class SdkQuadruped
{
public:
    SdkQuadruped(std::shared_ptr<ApiQuadruped> api, std::shared_ptr<SdkProtocolServer> sdk);
    void SetKeyIsExist(bool set) { isKeyExist_ = set; }

private:
    void RegisterSdkCall();
    std::shared_ptr<ApiQuadruped> api_;
    std::shared_ptr<SdkProtocolServer> sdk_;
    bool isKeyExist_{false};

    // API添加在下面
private:
    nlohmann::json SetLinearVelocity(const nlohmann::json& params);
    nlohmann::json GetLinearVelocity(const nlohmann::json& params);
    nlohmann::json GetLinearVelocityInfo(const nlohmann::json& params);
    std::pair<bool, nlohmann::json> PubLinearVelocity();

    nlohmann::json SetAngularVelocity(const nlohmann::json& params);
    nlohmann::json GetAngularVelocity(const nlohmann::json& params);
    nlohmann::json GetAngularVelocityInfo(const nlohmann::json& params);
    std::pair<bool, nlohmann::json> PubAngularVelocity();

    nlohmann::json SetPose(const nlohmann::json& params);
    nlohmann::json GetPose(const nlohmann::json& params);
    nlohmann::json GetPoseInfo(const nlohmann::json& params);
    std::pair<bool, nlohmann::json> PubPose();

    nlohmann::json SetHeight(const nlohmann::json& params);
    nlohmann::json GetHeight(const nlohmann::json& params);
    nlohmann::json GetHeightInfo(const nlohmann::json& params);
    std::pair<bool, nlohmann::json> PubHeight();

    nlohmann::json SetRunState(const nlohmann::json& params);
    nlohmann::json GetRunState(const nlohmann::json& params);
    nlohmann::json GetRunStateInfo(const nlohmann::json& params);
    std::pair<bool, nlohmann::json> PubRunState();

    nlohmann::json SetWalkMode(const nlohmann::json& params);
    nlohmann::json GetWalkMode(const nlohmann::json& params);
    nlohmann::json GetWalkModeInfo(const nlohmann::json& params);
    std::pair<bool, nlohmann::json> PubWalkMode();

    nlohmann::json SetGaitType(const nlohmann::json& params);
    nlohmann::json GetGaitType(const nlohmann::json& params);
    nlohmann::json GetGaitTypeInfo(const nlohmann::json& params);
    std::pair<bool, nlohmann::json> PubGaitType();

    nlohmann::json SetRobotCtrl(const nlohmann::json& params);
    nlohmann::json GetRobotState(const nlohmann::json& params);
    nlohmann::json GetRobotCtrlInfo(const nlohmann::json& params);
    std::pair<bool, nlohmann::json> PubRobotState();

    nlohmann::json SetHoldFlag(const nlohmann::json& params);
    nlohmann::json GetHoldFlag(const nlohmann::json& params);

    nlohmann::json SetLegMotorCmd(const nlohmann::json& params);
    nlohmann::json GetLegMotorData(const nlohmann::json& params);
    nlohmann::json GetImuData(const nlohmann::json& params);
    std::pair<bool, nlohmann::json> PubMotorData();
    std::pair<bool, nlohmann::json> PubImuData();

    nlohmann::json SetLoadMass(const nlohmann::json& params);
    nlohmann::json GetLoadMass(const nlohmann::json& params);
    nlohmann::json GetLoadMassInfo(const nlohmann::json& params);

    nlohmann::json SetDance(const nlohmann::json& params);

private:
    bool tmpHoldFlag = false;
    double preHeight_;
    nlohmann::json preRunState_;
    nlohmann::json preWalkMode_;
    nlohmann::json preGaitType_;
    nlohmann::json preRobotState_;
};
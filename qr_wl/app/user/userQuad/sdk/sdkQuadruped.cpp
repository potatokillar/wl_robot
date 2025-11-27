#include "sdkQuadruped.hpp"

#include "robotState.hpp"

using namespace nlohmann;
using namespace std;

/**
 * @description: 能用构造函数初始化的，就不要用Init再去初始化
 * @param ser
 * @param api
 * @return {}
 */
SdkQuadruped::SdkQuadruped(std::shared_ptr<ApiQuadruped> api, std::shared_ptr<SdkProtocolServer> sdk)
{
    api_ = api;
    sdk_ = sdk;
    RegisterSdkCall();
}

/**
 * @description: 向RPC服务器注册回调函数
 * @param rpcSer
 * @return {}
 */
void SdkQuadruped::RegisterSdkCall()
{
    sdk_->AddMethodCall("quadruped", "SetLinearVelocity", [this](json params) -> json { return this->SetLinearVelocity(params); });
    sdk_->AddMethodCall("quadruped", "GetLinearVelocity", [this](json params) -> json { return this->GetLinearVelocity(params); });
    sdk_->AddMethodCall("quadruped", "GetLinearVelocityInfo", [this](json params) -> json { return this->GetLinearVelocityInfo(params); });

    sdk_->AddMethodCall("quadruped", "SetAngularVelocity", [this](json params) -> json { return this->SetAngularVelocity(params); });
    sdk_->AddMethodCall("quadruped", "GetAngularVelocity", [this](json params) -> json { return this->GetAngularVelocity(params); });
    sdk_->AddMethodCall("quadruped", "GetAngularVelocityInfo", [this](json params) -> json { return this->GetAngularVelocityInfo(params); });

    sdk_->AddMethodCall("quadruped", "SetPose", [this](json params) -> json { return this->SetPose(params); });
    sdk_->AddMethodCall("quadruped", "GetPose", [this](json params) -> json { return this->GetPose(params); });
    sdk_->AddMethodCall("quadruped", "GetPoseInfo", [this](json params) -> json { return this->GetPoseInfo(params); });

    sdk_->AddMethodCall("quadruped", "SetHeight", [this](json params) -> json { return this->SetHeight(params); });
    sdk_->AddMethodCall("quadruped", "GetHeight", [this](json params) -> json { return this->GetHeight(params); });
    sdk_->AddMethodCall("quadruped", "GetHeightInfo", [this](json params) -> json { return this->GetHeightInfo(params); });

    sdk_->AddMethodCall("quadruped", "SetRunState", [this](json params) -> json { return this->SetRunState(params); });
    sdk_->AddMethodCall("quadruped", "GetRunState", [this](json params) -> json { return this->GetRunState(params); });
    sdk_->AddMethodCall("quadruped", "GetRunStateInfo", [this](json params) -> json { return this->GetRunStateInfo(params); });

    sdk_->AddMethodCall("quadruped", "SetWalkMode", [this](json params) -> json { return this->SetWalkMode(params); });
    sdk_->AddMethodCall("quadruped", "GetWalkMode", [this](json params) -> json { return this->GetWalkMode(params); });
    sdk_->AddMethodCall("quadruped", "GetWalkModeInfo", [this](json params) -> json { return this->GetWalkModeInfo(params); });

    sdk_->AddMethodCall("quadruped", "SetGaitType", [this](json params) -> json { return this->SetGaitType(params); });
    sdk_->AddMethodCall("quadruped", "GetGaitType", [this](json params) -> json { return this->GetGaitType(params); });
    sdk_->AddMethodCall("quadruped", "GetGaitTypeInfo", [this](json params) -> json { return this->GetGaitTypeInfo(params); });

    sdk_->AddMethodCall("quadruped", "SetRobotCtrl", [this](json params) -> json { return this->SetRobotCtrl(params); });
    sdk_->AddMethodCall("quadruped", "GetRobotState", [this](json params) -> json { return this->GetRobotState(params); });
    sdk_->AddMethodCall("quadruped", "GetRobotCtrlInfo", [this](json params) -> json { return this->GetRobotCtrlInfo(params); });

    sdk_->AddMethodCall("quadruped", "SetHoldFlag", [this](json params) -> json { return this->SetHoldFlag(params); });
    sdk_->AddMethodCall("quadruped", "GetHoldFlag", [this](json params) -> json { return this->GetHoldFlag(params); });

    sdk_->AddMethodCall("quadruped", "SetLoadMass", [this](json params) -> json { return this->SetLoadMass(params); });
    sdk_->AddMethodCall("quadruped", "GetLoadMass", [this](json params) -> json { return this->GetLoadMass(params); });
    sdk_->AddMethodCall("quadruped", "GetLoadMassInfo", [this](json params) -> json { return this->GetLoadMassInfo(params); });

    sdk_->AddMethodCall("quadruped", "SetDance", [this](json params) -> json { return this->SetDance(params); });

    // 二进制设置和返回
    sdk_->AddMethodCall("quadruped", "SetMotorCmd", [this](json params) -> json { return this->SetLegMotorCmd(params); });
    sdk_->AddMethodCall("quadruped", "GetData3", [this](json params) -> json { return this->GetLegMotorData(params); });
    sdk_->AddMethodCall("quadruped", "GetImuData", [this](json params) -> json { return this->GetImuData(params); });

    // 订阅支持，暂时控制上报速度
    sdk_->AddSubscribeCall("quadruped", "PubLinearVelocity", 10, [this]() -> std::pair<bool, json> { return this->PubLinearVelocity(); });
    sdk_->AddSubscribeCall("quadruped", "PubAngularVelocity", 10, [this]() -> std::pair<bool, json> { return this->PubAngularVelocity(); });
    sdk_->AddSubscribeCall("quadruped", "PubPose", 10, [this]() -> std::pair<bool, json> { return this->PubPose(); });
    sdk_->AddSubscribeCall("quadruped", "PubHeight", 10, [this]() -> std::pair<bool, json> { return this->PubHeight(); });
    sdk_->AddSubscribeCall("quadruped", "PubRunState", 10, [this]() -> std::pair<bool, json> { return this->PubRunState(); });
    sdk_->AddSubscribeCall("quadruped", "PubWalkMode", 10, [this]() -> std::pair<bool, json> { return this->PubWalkMode(); });
    sdk_->AddSubscribeCall("quadruped", "PubGaitType", 10, [this]() -> std::pair<bool, json> { return this->PubGaitType(); });
    sdk_->AddSubscribeCall("quadruped", "PubMotorData", 10, [this]() -> std::pair<bool, json> { return this->PubMotorData(); });
    sdk_->AddSubscribeCall("quadruped", "PubImuData", 10, [this]() -> std::pair<bool, json> { return this->PubImuData(); });
    sdk_->AddSubscribeCall("quadruped", "PubRobotState", 10, [this]() -> std::pair<bool, json> { return this->PubRobotState(); });
}

/**
 * @description: 设置腿部电机命令，二进制
 * @param params
 * @return {}
 */
nlohmann::json SdkQuadruped::SetLegMotorCmd(const nlohmann::json& params)
{
    if (isKeyExist_) {
        //  return ErrState2json(RetState::noSupport);
    }
    if (GetRobotCurState() != RobotState::jointCtrl) {
        return ErrState2json(RetState::noSupport);
    }

    RetState ret = RetState::ok;
    try {
        msg::qr::motor_cmd inData;
        vector<string> legName = {"legFR", "legFL", "legRR", "legRL"};

        for (int i = 0; i < 4; i++) {
            inData.leg[i][0].alpha = Json2Double(params[legName[i]]["abad"][0]);
            inData.leg[i][0].torq = Json2Double(params[legName[i]]["abad"][1]);
            inData.leg[i][0].blta = Json2Double(params[legName[i]]["abad"][2]);
            inData.leg[i][0].k2 = Json2Double(params[legName[i]]["abad"][3]);
            inData.leg[i][0].k1 = Json2Double(params[legName[i]]["abad"][4]);

            inData.leg[i][1].alpha = Json2Double(params[legName[i]]["hip"][0]);
            inData.leg[i][1].torq = Json2Double(params[legName[i]]["hip"][1]);
            inData.leg[i][1].blta = Json2Double(params[legName[i]]["hip"][2]);
            inData.leg[i][1].k2 = Json2Double(params[legName[i]]["hip"][3]);
            inData.leg[i][1].k1 = Json2Double(params[legName[i]]["hip"][4]);

            inData.leg[i][2].alpha = Json2Double(params[legName[i]]["knee"][0]);
            inData.leg[i][2].torq = Json2Double(params[legName[i]]["knee"][1]);
            inData.leg[i][2].blta = Json2Double(params[legName[i]]["knee"][2]);
            inData.leg[i][2].k2 = Json2Double(params[legName[i]]["knee"][3]);
            inData.leg[i][2].k1 = Json2Double(params[legName[i]]["knee"][4]);
        }

        MsgTrySend("qr::motor_cmd", inData);
        // api_->SendMotorCmd(inData);
    } catch (std::exception& e) {
        LOG_WARN("[SdkQuadruped] SetLegMotorCmd error: {}", e.what());
        ret = RetState::parseErr;
    }

    return ErrState2json(ret);
}

/**
 * @description: 获取腿部电机数据
 * @param params
 * @return {}
 */
nlohmann::json SdkQuadruped::GetLegMotorData(const nlohmann::json& params)
{
    if (GetRobotCurState() != RobotState::jointCtrl) {
        return ErrState2json(RetState::noSupport);
    }
    (void)params;
    // 接收电机数据
    msg::qr::motor_ret inData = api_->RecvMotorRet();
    json jSend;
    vector<string> legName = {"legFR", "legFL", "legRR", "legRL"};
    for (int i = 0; i < 4; i++) {
        jSend[legName[i]]["abad"].push_back(Double2Json(inData.leg[i][0].alpha));
        jSend[legName[i]]["abad"].push_back(Double2Json(inData.leg[i][0].torq));
        jSend[legName[i]]["abad"].push_back(Double2Json(inData.leg[i][0].blta));

        jSend[legName[i]]["hip"].push_back(Double2Json(inData.leg[i][1].alpha));
        jSend[legName[i]]["hip"].push_back(Double2Json(inData.leg[i][1].torq));
        jSend[legName[i]]["hip"].push_back(Double2Json(inData.leg[i][0].blta));

        jSend[legName[i]]["knee"].push_back(Double2Json(inData.leg[i][2].alpha));
        jSend[legName[i]]["knee"].push_back(Double2Json(inData.leg[i][2].torq));
        jSend[legName[i]]["knee"].push_back(Double2Json(inData.leg[i][2].blta));
    }
    return jSend;
}

std::pair<bool, nlohmann::json> SdkQuadruped::PubMotorData()
{
    json param;
    return {true, GetLegMotorData(param)};
}

/**
 * @description: 获取原始IMU数据
 * @param params
 * @return {}
 */
nlohmann::json SdkQuadruped::GetImuData(const nlohmann::json& params)
{
    (void)params;
    // cout << "1111" << endl;
    auto ret = MsgTryRecv<msg::imu_data>("qr::imuData", this);
    if (ret) {
        msg::imu_data inData = ret.value();
        json jSend;
        for (int i = 0; i < 3; i++) {
            jSend["gyro"].push_back(Double2Json(inData.gyro[i]));
            jSend["acc"].push_back(Double2Json(inData.acc[i]));
            jSend["ang"].push_back(Double2Json(inData.ang[i]));
            jSend["quat"].push_back(Double2Json(inData.quat[i]));
        }
        jSend["quat"].push_back(Double2Json(inData.quat[3]));
        return jSend;
    }

    return ErrState2json(RetState::noUpdate);
}

std::pair<bool, nlohmann::json> SdkQuadruped::PubImuData()
{
    json param;
    auto ret = GetImuData(param);
    if (ret.is_string() == false) {
        return {true, ret};
    }
    return {false, ret};
}

/**
 * @description: json设置线速度，z方向无效
 * @param params
 * @return {}
 */
json SdkQuadruped::SetLinearVelocity(const json& params)
{
    if (isKeyExist_) {
        return ErrState2json(RetState::noSupport);
    }
    double x, y;
    RetState ret = RetState::parseErr;
    try {
        x = Json2Double(params.at(0));
        y = Json2Double(params.at(1));
        ret = api_->SetLinearVelocity(x, y, 0);
    } catch (std::exception&) {
    }

    return ErrState2json(ret);
}
/**
 * @description: 获取线速度
 * @param params
 * @return {}
 */
json SdkQuadruped::GetLinearVelocity(const json& params)
{
    (void)params;
    auto [x, y, z] = api_->GetLinearVelocity();

    json j = {Double2Json(x), Double2Json(y), Double2Json(z)};
    return j;
}

/**
 * @description: 获取线速度信息
 * @param params
 * @return {}
 */
json SdkQuadruped::GetLinearVelocityInfo(const json& params)
{
    (void)params;

    auto [x, y, z] = api_->GetLinearVelocityRange();
    json j;
    j["range"]["x"].push_back(Double2Json(x.min));
    j["range"]["x"].push_back(Double2Json(x.max));
    j["range"]["y"].push_back(Double2Json(y.min));
    j["range"]["y"].push_back(Double2Json(y.max));
    j["range"]["z"].push_back(Double2Json(z.min));
    j["range"]["z"].push_back(Double2Json(z.max));
    return j;
}

/**
 * @description: 线速度订阅
 * @return {}
 */
std::pair<bool, json> SdkQuadruped::PubLinearVelocity()
{
    auto [x, y, z] = api_->GetLinearVelocity();
    json j = {Double2Json(x), Double2Json(y), Double2Json(z)};
    return {true, j};  // 必为true
}

/**
 * @description: json设置角速度，xy方向无效
 * @param params
 * @return {}
 */
json SdkQuadruped::SetAngularVelocity(const json& params)
{
    if (isKeyExist_) {
        return ErrState2json(RetState::noSupport);
    }
    double z;
    RetState ret = RetState::parseErr;
    try {
        z = Json2Double(params.at(2));
        ret = api_->SetAngularVelocity(0, 0, z);
    } catch (std::exception&) {
    }

    return ErrState2json(ret);
}

json SdkQuadruped::GetAngularVelocity(const json& params)
{
    (void)params;
    auto [x, y, z] = api_->GetAngularVelocity();

    json j = {Double2Json(x), Double2Json(y), Double2Json(z)};
    return j;
}

/**
 * @description: 获取角速度信息
 * @param params
 * @return {}
 */
json SdkQuadruped::GetAngularVelocityInfo(const json& params)
{
    (void)params;

    auto [x, y, z] = api_->GetAngularVelocityRange();

    json j;
    j["range"]["x"].push_back(Double2Json(x.min));
    j["range"]["x"].push_back(Double2Json(x.max));
    j["range"]["y"].push_back(Double2Json(y.min));
    j["range"]["y"].push_back(Double2Json(y.max));
    j["range"]["z"].push_back(Double2Json(z.min));
    j["range"]["z"].push_back(Double2Json(z.max));

    return j;
}

/**
 * @description: 角速度订阅
 * @return {}
 */
std::pair<bool, json> SdkQuadruped::PubAngularVelocity()
{
    auto [x, y, z] = api_->GetAngularVelocity();

    json j = {Double2Json(x), Double2Json(y), Double2Json(z)};
    return {true, j};
}

/**
 * @description: 设置姿态角
 * @param params
 * @return {}
 */
json SdkQuadruped::SetPose(const json& params)
{
    if (isKeyExist_) {
        return ErrState2json(RetState::noSupport);
    }
    double roll, pitch, yaw;
    RetState ret = RetState::parseErr;
    try {
        roll = Json2Double(params.at(0));
        pitch = Json2Double(params.at(1));
        yaw = Json2Double(params.at(2));
        ret = api_->SetPose(roll, pitch, yaw);
    } catch (std::exception&) {
    }

    return ErrState2json(ret);
}

json SdkQuadruped::GetPose(const json& params)
{
    (void)params;
    auto [roll, pitch, yaw] = api_->GetPose();

    json j = {Double2Json(roll), Double2Json(pitch), Double2Json(yaw)};
    return j;
}

/**
 * @description: 获取姿态信息
 * @param params
 * @return {}
 */
json SdkQuadruped::GetPoseInfo(const json& params)
{
    (void)params;
    auto [roll, pitch, yaw] = api_->GetPoseRange();

    json j;
    j["range"]["roll"].push_back(Double2Json(roll.min));
    j["range"]["roll"].push_back(Double2Json(roll.max));
    j["range"]["pitch"].push_back(Double2Json(pitch.min));
    j["range"]["pitch"].push_back(Double2Json(pitch.max));
    j["range"]["yaw"].push_back(Double2Json(yaw.min));
    j["range"]["yaw"].push_back(Double2Json(yaw.max));
    return j;
}

/**
 * @description: 姿态角订阅
 * @return {}
 */
std::pair<bool, json> SdkQuadruped::PubPose()
{
    auto [roll, pitch, yaw] = api_->GetPose();

    json j = {Double2Json(roll), Double2Json(pitch), Double2Json(yaw)};
    return {true, j};
}

/**
 * @description: 设置高度
 * @param params
 * @return {}
 */
json SdkQuadruped::SetHeight(const json& params)
{
    if (isKeyExist_) {
        return ErrState2json(RetState::noSupport);
    }
    return ErrState2json(api_->SetHeight(Json2Double(params)));
}

json SdkQuadruped::GetHeight(const json& params)
{
    (void)params;
    double height = api_->GetHeight();
    LOG_DEBUG("GetHeight: {}", height);
    return Double2Json(height);
}

json SdkQuadruped::GetHeightInfo(const json& params)
{
    LOG_DEBUG("GetHeightInfo");
    (void)params;
    auto height = api_->GetHeightRange();

    json j;
    j["range"].push_back(Double2Json(height.min));
    j["range"].push_back(Double2Json(height.max));

    return j;
}

std::pair<bool, json> SdkQuadruped::PubHeight()
{
    double height = api_->GetHeight();
    if (fabs(height - preHeight_) < 0.01) {
        return {false, height};
    }
    preHeight_ = height;
    return {true, Double2Json(height)};
}

json SdkQuadruped::SetRunState(const json& params)
{
    LOG_DEBUG("SetRunState {}", params);
    if (isKeyExist_) {
        return ErrState2json(RetState::noSupport);
    }
    RetState err = RetState::parseErr;
    if (params == "lie") {
        err = api_->SetRunState(RunState::lie);
    } else if (params == "stand") {
        err = api_->SetRunState(RunState::stand);
    } else if (params == "walk") {
        err = api_->SetRunState(RunState::walk);
    }
    return ErrState2json(err);
}

json SdkQuadruped::GetRunState(const json& params)
{
    (void)params;
    RunState sta = api_->GetRunState();
    // LOG_DEBUG("GetRunState {}", Enum2Num(sta));
    if (sta == RunState::lie) {
        return "lie";
    } else if (sta == RunState::stand) {
        return "stand";
    } else if (sta == RunState::walk) {
        return "walk";
    }
    return "switching";
}

json SdkQuadruped::GetRunStateInfo(const json& params)
{
    (void)params;
    vector<RunState> state = api_->GetRunStateRange();

    json j;
    for (const auto& var : state) {
        switch (var) {
            case RunState::lie:
                j["restrict"].push_back("lie");
                break;
            case RunState::stand:
                j["restrict"].push_back("stand");
                break;
            case RunState::walk:
                j["restrict"].push_back("walk");
                break;
            default:
                break;
        }
    }

    return j;
}

std::pair<bool, json> SdkQuadruped::PubRunState()
{
    json param;
    auto ret = GetRunState(param);

    if (ret == preRunState_) {
        return {false, ret};
    }
    preRunState_ = ret;
    return {true, ret};
}

json SdkQuadruped::SetWalkMode(const json& params)
{
    LOG_DEBUG("SetRunMode {}", params);
    if (isKeyExist_) {
        return ErrState2json(RetState::noSupport);
    }
    RetState ret = RetState::noSupport;
    if (params == "normal") {
        ret = api_->SetWalkMode(WalkMode::normal);
    } else if (params == "aiClimb") {
        ret = api_->SetWalkMode(WalkMode::aiClimb);
    } else if (params == "aiNormal") {
        ret = api_->SetWalkMode(WalkMode::aiNormal);
    }
    return ErrState2json(ret);
}

json SdkQuadruped::GetWalkMode(const json& params)
{
    (void)params;
    WalkMode mode = api_->GetWalkMode();
    // LOG_DEBUG("GetWalkMode {}", Enum2Num(mode));
    if (mode == WalkMode::normal) {
        return "normal";
    } else if (mode == WalkMode::climb) {
        return "climb";
    } else if (mode == WalkMode::light) {
        return "light";
    }
    return "normal";
}
json SdkQuadruped::GetWalkModeInfo(const json& params)
{
    (void)params;
    vector<WalkMode> mode = api_->GetWalkModeRange();

    json j;
    for (const auto& var : mode) {
        switch (var) {
            case WalkMode::normal:
                j["restrict"].push_back("normal");
                break;
            case WalkMode::climb:
                j["restrict"].push_back("climb");
                break;
            case WalkMode::light:
                j["restrict"].push_back("light");
                break;
            default:
                break;
        }
    }

    return j;
}

std::pair<bool, json> SdkQuadruped::PubWalkMode()
{
    json param;
    auto ret = GetWalkMode(param);

    if (ret == preWalkMode_) {
        return {false, ret};
    }
    preWalkMode_ = ret;
    return {true, ret};
}

json SdkQuadruped::SetGaitType(const json& params)
{
    LOG_DEBUG("SetGaitType {}", params);
    if (isKeyExist_) {
        return ErrState2json(RetState::noSupport);
    }
    if (params == "tort") {
        return api_->SetGaitType(GaitType::tort);
    }
    return ErrState2json(RetState::noSupport);
}

json SdkQuadruped::GetGaitType(const json& params)
{
    (void)params;
    LOG_DEBUG("GetGaitType");
    GaitType gait = api_->GetGaitType();
    if (gait == GaitType::tort) {
        return "tort";
    }
    return "null";
}

json SdkQuadruped::GetGaitTypeInfo(const json& params)
{
    (void)params;
    vector<GaitType> gait = api_->GetGaitTypeRange();

    json j;
    j["restrict"] = "tort";

    return j;
}

// 当下不支持
std::pair<bool, json> SdkQuadruped::PubGaitType()
{
    json jSend;

    return {false, jSend};
}

/**
 * @description: 设置机器人控制行为
 * @param params
 * @return {}
 */
// todo 返回值要确认
json SdkQuadruped::SetRobotCtrl(const json& params)
{
    if (isKeyExist_) {
        return ErrState2json(RetState::noSupport);
    }

    try {
        // LOG_DEBUG("SetRobotCtrl:{}", params);
        if (params == "start") {
            api_->Start(QuadBootMode::qr);
        } else if (params == "stop") {
            api_->Stop();
        } else if (params == "enterEmgstop") {
            api_->SetEmergStop(true);
        } else if (params == "exitEmgstop") {
            api_->SetEmergStop(false);
        } else if (params == "lowStart") {
            api_->Start(QuadBootMode::joint);
        }
    } catch (const std::exception& e) {
        return ErrState2json(RetState::noSupport);
    }
    return ErrState2json(RetState::ok);
}

json SdkQuadruped::GetRobotState(const json& params)
{
    (void)params;
    if (GetRobotCurState() == RobotState::running) {
        return "running";
    } else if (GetRobotCurState() == RobotState::standby) {
        return "standby";
    } else if (GetRobotCurState() == RobotState::emgstop) {
        return "emgstop";
    }
    return "error";
}
json SdkQuadruped::GetRobotCtrlInfo(const json& params)
{
    (void)params;
    vector<string> motor = {"start", "stop"};

    json j;
    j["restrict"] = motor;

    return j;
}

std::pair<bool, json> SdkQuadruped::PubRobotState()
{
    json param;
    auto ret = GetRobotState(param);

    if (ret == preRobotState_) {
        return {false, ret};
    }
    preRobotState_ = ret;
    return {true, ret};
}

nlohmann::json SdkQuadruped::SetHoldFlag(const nlohmann::json& params)
{
    LOG_DEBUG("SetHoldFlag {}", to_string(params));
    try {
        return ErrState2json(api_->SetHoldFlag(params));
    } catch (const std::exception& e) {
        return ErrState2json(RetState::parseErr);
    }
}
nlohmann::json SdkQuadruped::GetHoldFlag(const nlohmann::json& params)
{
    (void)params;
    return tmpHoldFlag;
}

nlohmann::json SdkQuadruped::SetLoadMass(const nlohmann::json& params)
{
    try {
        return api_->SetLoadMass(Json2Double(params));
    } catch (const std::exception& e) {
        return ErrState2json(RetState::parseErr);
    }
}
nlohmann::json SdkQuadruped::GetLoadMass(const nlohmann::json& params)
{
    (void)params;
    return Double2Json(api_->GetLoadMass());
}

nlohmann::json SdkQuadruped::GetLoadMassInfo(const nlohmann::json& params)
{
    LOG_DEBUG("GetLoadMassInfo");
    (void)params;
    auto range = api_->GetLoadMassRange();

    json j;
    j["range"].push_back(Double2Json(range.min));
    j["range"].push_back(Double2Json(range.max));

    return j;
}

nlohmann::json SdkQuadruped::SetDance(const nlohmann::json& params)
{
    try {
        return ErrState2json(api_->SetDance(params));
    } catch (const std::exception& e) {
        return ErrState2json(RetState::parseErr);
    }
}

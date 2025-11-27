
#include "sdkArm.hpp"

#include "baseline.hpp"
#include "deviceCustomParam.hpp"
#include "netMsg.hpp"
#include "robotMedia.hpp"
#include "robotState.hpp"

using namespace nlohmann;
using namespace std;
using namespace arm;

std::map<arm::ArmState, std::string_view> ARMSTATE_JSON = {{ArmState::noRun, "noRun"},
                                                           {ArmState::init, "init"},
                                                           {ArmState::noReady, "noReady"},
                                                           {ArmState::ready, "ready"},
                                                           {ArmState::running, "running"},
                                                           {ArmState::pause, "pause"},
                                                           {ArmState::complete, "complete"},
                                                           {ArmState::interrupt, "interrupt"},
                                                           {ArmState::error, "error"}};

/**
 * @description: 能用构造函数初始化的，就不要用Init再去初始化
 * @param ser
 * @param api
 * @return {}
 */
SdkArm::SdkArm(std::shared_ptr<ApiArm> api, std::shared_ptr<SdkProtocolServer> sdk) : api_(api), sdk_(sdk) { RegisterSdkCall(); }

/**
 * @description: 向sdk服务器注册回调函数
 * @param rpcSer
 * @return {}
 */
void SdkArm::RegisterSdkCall()
{
    sdk_->AddMethodCall("arm", "SetMoveLTask", [this](json params) -> json { return this->SetMoveLTask(params); });
    sdk_->AddMethodCall("arm", "SetMoveJTask", [this](json params) -> json { return this->SetMoveJTask(params); });
    sdk_->AddMethodCall("arm", "WaitTaskComplete", [this](json params) -> json { return this->WaitTaskComplete(params); });

    sdk_->AddMethodCall("arm", "SetCtrlState", [this](json params) -> json { return this->SetCtrlState(params); });

    sdk_->AddMethodCall("arm", "GetToolNameList", [this](json params) -> json { return this->GetToolNameList(params); });
    sdk_->AddMethodCall("arm", "SetToolName", [this](json params) -> json { return this->SetToolName(params); });
    sdk_->AddMethodCall("arm", "GetToolName", [this](json params) -> json { return this->GetToolName(params); });
    sdk_->AddMethodCall("arm", "SetToolValue", [this](json params) -> json { return this->SetToolValue(params); });
    sdk_->AddMethodCall("arm", "GetToolValue", [this](json params) -> json { return this->GetToolValue(params); });

    sdk_->AddMethodCall("arm", "GetNowAngle", [this](json params) -> json { return this->GetNowAngle(params); });
    sdk_->AddMethodCall("arm", "GetNowCart", [this](json params) -> json { return this->GetNowCart(params); });

    sdk_->AddMethodCall("arm", "GetArmState", [this](json params) -> json { return this->GetArmState(params); });
    sdk_->AddMethodCall("arm", "GetArmInfo", [this](json params) -> json { return this->GetArmInfo(params); });

    sdk_->AddMethodCall("arm", "GetFkine6", [this](json params) -> json { return this->GetFkine6(params); });
    sdk_->AddMethodCall("arm", "GetIkine6", [this](json params) -> json { return this->GetIkine6(params); });

    sdk_->AddMethodCall("arm", "GetFkine7", [this](json params) -> json { return this->GetFkine7(params); });
    sdk_->AddMethodCall("arm", "GetIkine7", [this](json params) -> json { return this->GetIkine7(params); });

    sdk_->AddMethodCall("arm", "SetIoOutState", [this](json params) -> json { return this->SetIoOutState(params); });
    sdk_->AddMethodCall("arm", "GetIoOutState", [this](json params) -> json { return this->GetIoOutState(params); });
    sdk_->AddMethodCall("arm", "GetIoInState", [this](json params) -> json { return this->GetIoInState(params); });

    sdk_->AddMethodCall("arm", "SetPositionLimit", [this](json params) -> json { return this->SetPositionLimit(params); });
    sdk_->AddMethodCall("arm", "GetPositionLimit", [this](json params) -> json { return this->GetPositionLimit(params); });

    sdk_->AddMethodCall("arm", "SetCollisionLevel", [this](json params) -> json { return this->SetCollisionLevel(params); });
    sdk_->AddMethodCall("arm", "GetCollisionLevel", [this](json params) -> json { return this->GetCollisionLevel(params); });

    sdk_->AddMethodCall("arm", "SetJointSpeedLimit", [this](json params) -> json { return this->SetJointSpeedLimit(params); });
    sdk_->AddMethodCall("arm", "GetJointSpeedLimit", [this](json params) -> json { return this->GetJointSpeedLimit(params); });

    sdk_->AddMethodCall("arm", "SetJointAccLimit", [this](json params) -> json { return this->SetJointAccLimit(params); });
    sdk_->AddMethodCall("arm", "GetJointAccLimit", [this](json params) -> json { return this->GetJointAccLimit(params); });

    sdk_->AddMethodCall("arm", "SetDragMode", [this](json params) -> json { return this->SetDragMode(params); });
    sdk_->AddMethodCall("arm", "GetDragMode", [this](json params) -> json { return this->GetDragMode(params); });

    sdk_->AddMethodCall("arm", "SetRapidrate", [this](json params) -> json { return this->SetRapidrate(params); });
    sdk_->AddMethodCall("arm", "GetRapidrate", [this](json params) -> json { return this->GetRapidrate(params); });

    sdk_->AddMethodCall("arm", "SetReset", [this](json params) -> json { return this->SetReset(params); });
    sdk_->AddMethodCall("arm", "GetReset", [this](json params) -> json { return this->GetReset(params); });

    sdk_->AddMethodCall("arm", "SetGripper", [this](json params) -> json { return this->SetGripper(params); });
    // 订阅支持
    sdk_->AddSubscribeCall("arm", "PubArmState", 20, [this]() -> std::pair<bool, json> { return this->PubArmState(); });
    sdk_->AddSubscribeCall("arm", "PubArmInfo", 20, [this]() -> std::pair<bool, json> { return this->PubArmInfo(); });

    sdk_->AddSubscribeCall("arm", "PubNowAngle", 20, [this]() -> std::pair<bool, json> { return this->PubNowAngle(); });
    sdk_->AddSubscribeCall("arm", "PubNowCart", 20, [this]() -> std::pair<bool, json> { return this->PubNowCart(); });

    sdk_->AddSubscribeCall("arm", "PubIoOutState", 20, [this]() -> std::pair<bool, json> { return this->PubIoOutState(); });
    sdk_->AddSubscribeCall("arm", "PubIoInState", 20, [this]() -> std::pair<bool, json> { return this->PubIoInState(); });
}

void SdkArm::Start()
{
    if (api_->IsRun() == false) {
        // api_->Start();
    }
}

nlohmann::json SdkArm::SetMoveJTask(const nlohmann::json& params)
{
    json jRet;
    try {
        auto jGoal = params.at("goal");
        vector<vector<double>> pos;
        // vector<Vec6<double>> pos6;
        // vector<Vec7<double>> pos7;
        for (size_t i = 0; i < jGoal.size(); i++) {
            // 必须是6轴或7轴
            vector<double> posone;
            for (size_t j = 0; j < jGoal.at(i).size() && j < 7; j++) {
                posone.push_back(Json2Double(jGoal.at(i).at(j)));
            }
            pos.push_back(posone);
        }
        auto jMode = params.at("mode");
        MoveMode mode;
        if (jMode == "abs") {
            mode = MoveMode::abs;
        } else {
            mode = MoveMode::incr;
        }

        Result<u32> ret;

        // 不存在只存在acc不存在speed的协议
        if (params.contains("speed")) {
            auto jSpeed = Json2Double(params.at("speed"));
            if (params.contains("acc")) {
                auto jAcc = Json2Double(params.at("acc"));
                if (pos.empty() == false) {
                    ret = api_->SetMoveJTaskNoBlock(pos, mode, jSpeed, jAcc);
                } else {
                    return ErrState2json(RetState::parseErr);
                }
            } else {
                if (pos.empty() == false) {
                    ret = api_->SetMoveJTaskNoBlock(pos, mode, jSpeed);
                } else {
                    return ErrState2json(RetState::parseErr);
                }
            }
        } else {
            if (pos.empty() == false) {
                ret = api_->SetMoveJTaskNoBlock(pos, mode);
            } else {
                return ErrState2json(RetState::parseErr);
            }
        }

        if (ret.first == RetState::ok) {
            jRet = ret.second;
        } else {
            jRet = ErrState2json(ret.first);
        }
    } catch (std::exception&) {
        jRet = ErrState2json(RetState::parseErr);
    }
    return jRet;
}

nlohmann::json SdkArm::SetMoveLTask(const nlohmann::json& params)
{
    //  LOG_DEBUG("SetMoveLTask");
    json jRet;
    try {
        auto jGoal = params.at("goal");
        vector<Pose> poss;
        for (size_t i = 0; i < jGoal.size(); i++) {
            Pose pos;
            pos.tran_.x = Json2Double(jGoal.at(i).at(0));
            pos.tran_.y = Json2Double(jGoal.at(i).at(1));
            pos.tran_.z = Json2Double(jGoal.at(i).at(2));
            pos.rpy_.rx = Json2Double(jGoal.at(i).at(3));
            pos.rpy_.ry = Json2Double(jGoal.at(i).at(4));
            pos.rpy_.rz = Json2Double(jGoal.at(i).at(5));
            poss.push_back(pos);
        }
        auto jMode = params.at("mode");

        MoveMode mode;
        if (jMode == "abs") {
            mode = MoveMode::abs;
        } else {
            mode = MoveMode::incr;
        }

        Result<u32> ret;
        if (params.contains("speed")) {
            auto jSpeed = Json2Double(params.at("speed"));
            if (params.contains("acc")) {
                auto jAcc = Json2Double(params.at("acc"));
                ret = api_->SetMoveLTaskNoBlock(poss, mode, jSpeed, jAcc);
            } else {
                ret = api_->SetMoveLTaskNoBlock(poss, mode, jSpeed);
            }
        } else {
            ret = api_->SetMoveLTaskNoBlock(poss, mode);
        }
        if (ret.first == RetState::ok) {
            jRet = ret.second;
        } else {
            jRet = ErrState2json(ret.first);
        }

    } catch (std::exception&) {
        jRet = ErrState2json(RetState::parseErr);
    }
    return jRet;
}

nlohmann::json SdkArm::WaitTaskComplete(const nlohmann::json& params)
{
    u32 ms;
    try {
        ms = params;
        return ErrState2json(api_->WaitMoveTaskComplete(ms));
    } catch (std::exception&) {
    }
    return ErrState2json(RetState::parseErr);
}

nlohmann::json SdkArm::SetCtrlState(const nlohmann::json& params)
{
    Start();
    RetState ret = RetState::parseErr;
    try {
        string cmd = to_string(params);
        cmd = cmd.substr(1, cmd.size() - 2);  // 下发的字符串是包括双引号的，计算hash时要去掉
        switch (BKDRHash(cmd.c_str())) {
            case "start"_hash:
                ret = api_->SetCtrlState(arm::ArmCtrl::start);
                break;
            case "stop"_hash:
                ret = api_->SetCtrlState(arm::ArmCtrl::stop);
                break;
            case "pause"_hash:
                ret = api_->SetCtrlState(arm::ArmCtrl::pause);
                break;
            case "enable"_hash:
                ret = api_->SetCtrlState(arm::ArmCtrl::enable);
                break;
            case "disable"_hash:
                ret = api_->SetCtrlState(arm::ArmCtrl::disable);
                break;
            case "emgStop"_hash:
                ret = api_->SetCtrlState(arm::ArmCtrl::emgStop);
                break;
            default:
                ret = RetState::noSupport;
                break;
        }
    } catch (std::exception&) {
    }
    return ErrState2json(ret);
}

nlohmann::json SdkArm::GetToolNameList(const nlohmann::json& params)
{
    (void)params;
    json jSend;
    auto tools = api_->GetToolNameList();
    if (tools.empty() == false) {
        for (const auto& v : tools) {
            jSend.push_back(v);
        }
    } else {
        jSend = ErrState2json(RetState::noExist);
    }
    return jSend;
}

nlohmann::json SdkArm::SetToolName(const nlohmann::json& params)
{
    json jRet;
    try {
        jRet = ErrState2json(api_->SetToolName(params));
    } catch (std::exception&) {
        jRet = ErrState2json(RetState::parseErr);
    }
    return jRet;
}

nlohmann::json SdkArm::GetToolName(const nlohmann::json& params)
{
    (void)params;
    json jSend;
    auto tool = api_->GetToolName();
    if (tool.empty() == false) {
        jSend = tool;
    } else {
        jSend = ErrState2json(RetState::noExist);
    }
    return jSend;
}

nlohmann::json SdkArm::SetToolValue(const nlohmann::json& params)
{
    json jRet;
    try {
        auto jPos = params.at("pos");
        Pose pos;
        pos.tran_.x = Json2Double(jPos.at(0));
        pos.tran_.y = Json2Double(jPos.at(1));
        pos.tran_.z = Json2Double(jPos.at(2));
        pos.rpy_.rx = Json2Double(jPos.at(3));
        pos.rpy_.ry = Json2Double(jPos.at(4));
        pos.rpy_.rz = Json2Double(jPos.at(5));

        string name = params.at("name");

        jRet = ErrState2json(api_->SetToolValue(name, pos));
    } catch (std::exception&) {
        jRet = ErrState2json(RetState::parseErr);
    }
    return jRet;
}

nlohmann::json SdkArm::GetToolValue(const nlohmann::json& params)
{
    Start();
    LOG_DEBUG("GetToolValue");
    json jRet;
    try {
        std::string name = params;
        auto val = api_->GetToolValue(name);
        if (val.first == RetState::ok) {
            jRet.push_back(Double2Json(val.second.tran_.x));
            jRet.push_back(Double2Json(val.second.tran_.y));
            jRet.push_back(Double2Json(val.second.tran_.z));
            jRet.push_back(Double2Json(val.second.rpy_.rx));
            jRet.push_back(Double2Json(val.second.rpy_.ry));
            jRet.push_back(Double2Json(val.second.rpy_.rz));
        } else {
            jRet = ErrState2json(RetState::noExist);
        }

    } catch (std::exception&) {
        jRet = ErrState2json(RetState::parseErr);
    }
    return jRet;
}

nlohmann::json SdkArm::GetNowAngle(const nlohmann::json& params)
{
    (void)params;
    // LOG_DEBUG("GetNowAngle");
    json jRet;
    try {
        auto angle = api_->GetNowAngle();
        if (angle.empty() == false) {
            for (const auto& v : angle) {
                jRet.push_back(Double2Json(v));
            }
        }
    } catch (std::exception&) {
        jRet = ErrState2json(RetState::noExist);
    }
    return jRet;
}

std::pair<bool, nlohmann::json> SdkArm::PubNowAngle()
{
    json jRet;
    auto angle = api_->GetNowAngle();
    if (angle.empty() == false) {
        for (const auto& v : angle) {
            jRet.push_back(Double2Json(v));
        }
        return {true, jRet};
    }
    return {false, ErrState2json(RetState::noExist)};
}

nlohmann::json SdkArm::GetNowCart(const nlohmann::json& params)
{
    (void)params;
    // LOG_DEBUG("GetNowCart");
    json jRet;
    try {
        auto angle = api_->GetNowCart();  // todo
        jRet.push_back(Double2Json(angle.tran_.x));
        jRet.push_back(Double2Json(angle.tran_.y));
        jRet.push_back(Double2Json(angle.tran_.z));
        jRet.push_back(Double2Json(angle.rpy_.rx));
        jRet.push_back(Double2Json(angle.rpy_.ry));
        jRet.push_back(Double2Json(angle.rpy_.rz));
    } catch (std::exception&) {
        jRet = ErrState2json(RetState::parseErr);
    }
    return jRet;
}

std::pair<bool, nlohmann::json> SdkArm::PubNowCart()
{
    json jRet;
    auto sta = api_->GetArmState();
    if (sta == ArmState::noRun) {
        return {false, jRet};
    }

    auto cart = api_->GetNowCart();
    jRet.push_back(Double2Json(cart.tran_.x));
    jRet.push_back(Double2Json(cart.tran_.y));
    jRet.push_back(Double2Json(cart.tran_.z));
    jRet.push_back(Double2Json(cart.rpy_.rx));
    jRet.push_back(Double2Json(cart.rpy_.ry));
    jRet.push_back(Double2Json(cart.rpy_.rz));
    return {true, jRet};
}

nlohmann::json SdkArm::GetArmState(const nlohmann::json& params)
{
    (void)params;
    // LOG_DEBUG("GetArmState");
    json jRet;
    try {
        auto sta = api_->GetArmState();
        jRet = ARMSTATE_JSON.at(sta);
    } catch (std::exception&) {
        jRet = ErrState2json(RetState::parseErr);
    }
    return jRet;
}

std::pair<bool, json> SdkArm::PubArmState()
{
    json jRet{RetState::noExist};
    auto sta = api_->GetArmState();

    if (sta != preArmSta_) {
        preArmSta_ = sta;
        try {
            jRet = ARMSTATE_JSON.at(sta);
            return {true, jRet};
        } catch (std::exception&) {
        }
    }

    return {false, jRet};
}

nlohmann::json SdkArm::GetFkine6(const nlohmann::json& params)
{
    // LOG_DEBUG("GetFkine");
    json jRet;
    try {
        std::array<double, 6> joint;
        for (int i = 0; i < 6; i++) {
            joint[i] = Json2Double(params.at("joint").at(i));
        }

        vector<double> jointV;
        jointV.resize(6);
        for (int i = 0; i < 6; i++) {
            jointV[i] = joint.at(i);
        }

        auto ret = api_->GetFkine6(jointV);
        jRet.push_back(Double2Json(ret.tran_.x));
        jRet.push_back(Double2Json(ret.tran_.y));
        jRet.push_back(Double2Json(ret.tran_.z));
        jRet.push_back(Double2Json(ret.rpy_.rx));
        jRet.push_back(Double2Json(ret.rpy_.ry));
        jRet.push_back(Double2Json(ret.rpy_.rz));
    } catch (std::exception&) {
        jRet = ErrState2json(RetState::parseErr);
    }
    return jRet;
}

nlohmann::json SdkArm::GetIkine6(const nlohmann::json& params)
{
    json jRet;
    try {
        std::array<double, 6> joint;
        for (int i = 0; i < 6; i++) {
            joint.at(i) = Json2Double(params.at("FK")[i]);
        }
        Pose jointV;
        jointV.tran_.x = joint[0];
        jointV.tran_.y = joint[1];
        jointV.tran_.z = joint[2];
        jointV.rpy_.rx = joint[3];
        jointV.rpy_.ry = joint[4];
        jointV.rpy_.rz = joint[5];

        auto nowAngle = api_->GetNowAngle();
        if (nowAngle.empty()) {
            return ErrState2json(RetState::noSupport);
        }

        // vector<double> ang;
        // for (int i = 0; i < 6; i++) {
        //     ang[i] = nowAngle.at(i);
        // }

        auto ret = api_->GetIkine6(jointV, nowAngle);
        if (ret.first == true) {
            for (int i = 0; i < 6; i++) {
                jRet.push_back(Double2Json(ret.second(i)));
            }
        } else {
            LOG_DEBUG_SDK("Ik6 failed! Out of Range!");
            jRet = ErrState2json(RetState::outRange);
        }

    } catch (std::exception&) {
        jRet = ErrState2json(RetState::parseErr);
    }
    return jRet;
}

nlohmann::json SdkArm::GetFkine7(const nlohmann::json& params)
{
    LOG_DEBUG_SDK("GetFkine7");
    json jRet;
    try {
        vector<double> jointV;
        for (int i = 0; i < 7; i++) {
            jointV.push_back(Json2Double(params.at(i)));
        }

        auto ret = api_->GetFkine7(jointV);
        jRet.push_back(Double2Json(ret.first.tran_.x));
        jRet.push_back(Double2Json(ret.first.tran_.y));
        jRet.push_back(Double2Json(ret.first.tran_.z));
        jRet.push_back(Double2Json(ret.first.rpy_.rx));
        jRet.push_back(Double2Json(ret.first.rpy_.ry));
        jRet.push_back(Double2Json(ret.first.rpy_.rz));
        jRet.push_back(Double2Json(ret.second));
    } catch (std::exception&) {
        jRet = ErrState2json(RetState::parseErr);
    }
    return jRet;
}
nlohmann::json SdkArm::GetIkine7(const nlohmann::json& params)
{
    LOG_DEBUG_SDK("GetIkine7");
    json jRet;
    try {
        Pose jointV;
        jointV.tran_.x = Json2Double(params.at(0));
        jointV.tran_.y = Json2Double(params.at(1));
        jointV.tran_.z = Json2Double(params.at(2));
        jointV.rpy_.rx = Json2Double(params.at(3));
        jointV.rpy_.ry = Json2Double(params.at(4));
        jointV.rpy_.rz = Json2Double(params.at(5));
        double phi = Json2Double(params.at(6));

        auto nowAngle = api_->GetNowAngle();
        if (nowAngle.empty()) {
            return ErrState2json(RetState::noSupport);
        }

        auto ret = api_->GetIkine7(jointV, phi, nowAngle);
        if (ret.first == true) {
            for (int i = 0; i < 7; i++) {
                jRet.push_back(Double2Json(ret.second(i)));
            }
        } else {
            LOG_DEBUG_SDK("Ik7 failed! Out of Range!");
            jRet = ErrState2json(RetState::outRange);
        }

    } catch (std::exception&) {
        jRet = ErrState2json(RetState::parseErr);
    }
    return jRet;
}

nlohmann::json SdkArm::SetIoOutState(const nlohmann::json& params)
{
    json jRet;

    try {
        string name = params.at("name");
        u32 sta = params.at("sta");  // 库不支持直接转
        std::bitset<32> set = sta;

        // name是panel，tool，extra三种
        auto ret = api_->SetIoOutState(name, set);
        jRet = ErrState2json(ret);
    } catch (std::exception&) {
        jRet = ErrState2json(RetState::parseErr);
    }

    return jRet;
}

nlohmann::json SdkArm::GetIoOutState(const nlohmann::json& params)
{
    (void)params;
    json jRet;
    try {
        auto ret = api_->GetIoOutState(params);
        jRet = ret.to_ulong();
    } catch (std::exception&) {
        jRet = ErrState2json(RetState::parseErr);
    }
    return jRet;
}

nlohmann::json SdkArm::GetIoInState(const nlohmann::json& params)
{
    (void)params;
    json jRet;
    try {
        auto ret = api_->GetIoInState(params);
        jRet = ret.to_ulong();
    } catch (std::exception&) {
        jRet = ErrState2json(RetState::parseErr);
    }
    return jRet;
}

std::pair<bool, json> SdkArm::PubIoOutState()
{
    json jRet;
    std::map<std::string, std::bitset<32>> sta;

    sta["panel"] = api_->GetIoOutState("panel");
    sta["tool"] = api_->GetIoOutState("tool");
    sta["extra"] = api_->GetIoOutState("extra");

    try {
        if (sta != preIoOutSta_) {
            preIoOutSta_ = sta;
            LOG_DEBUG("PubIoOutState");
            for (const auto& v : sta) {
                jRet[v.first] = v.second.to_ulong();
            }
            return {true, jRet};
        }

    } catch (std::exception&) {
        jRet = ErrState2json(RetState::parseErr);
    }
    return {false, jRet};
}

std::pair<bool, json> SdkArm::PubIoInState()
{
    json jRet;
    std::map<std::string, std::bitset<32>> sta;

    sta["panel"] = api_->GetIoInState("panel");
    sta["tool"] = api_->GetIoInState("tool");
    sta["extra"] = api_->GetIoInState("extra");

    try {
        if (sta != preIoInSta_) {
            preIoInSta_ = sta;
            for (const auto& v : sta) {
                jRet[v.first] = v.second.to_ulong();
            }
            return {true, jRet};
        }

    } catch (std::exception&) {
        jRet = ErrState2json(RetState::parseErr);
    }
    return {false, jRet};
}

nlohmann::json SdkArm::SetPositionLimit(const nlohmann::json& params)
{
    try {
        std::vector<ValueRange<double>> range;
        for (size_t i = 0; i < params.size(); i++) {
            ValueRange<double> val;
            val.min = Json2Double(params.at(i).at(0));
            val.max = Json2Double(params.at(i).at(1));
            range.push_back(val);
        }
        auto ret = api_->SetPositionLimit(range);
        return ErrState2json(ret);
    } catch (std::exception&) {
        return ErrState2json(RetState::parseErr);
    }
}
nlohmann::json SdkArm::GetPositionLimit(const nlohmann::json& params)
{
    (void)params;
    json jRet;
    auto [ret, limit] = api_->GetPositionLimit();
    if (ret == RetState::ok) {
        for (size_t i = 0; i < limit.size(); i++) {
            // array<double, 2> elem{limit[i].min, limit[i].max};
            array<std::string, 2> elem{Double2Json(limit[i].min), Double2Json(limit[i].max)};
            jRet.push_back(elem);
        }
    } else {
        jRet = ErrState2json(RetState::noExist);
    }
    return jRet;
}

nlohmann::json SdkArm::SetCollisionLevel(const nlohmann::json& params)
{
    try {
        auto ret = api_->SetCollisionLevel(params);
        return ErrState2json(ret);
    } catch (std::exception&) {
        return ErrState2json(RetState::parseErr);
    }
}

nlohmann::json SdkArm::GetCollisionLevel(const nlohmann::json& params)
{
    (void)params;
    json jRet;
    auto ret = api_->GetCollisionLevel();
    if (ret.first == RetState::ok) {
        jRet = ret.second;
    } else {
        jRet = ErrState2json(ret.first);
    }
    return jRet;
}

nlohmann::json SdkArm::GetArmInfo(const nlohmann::json& params)
{
    (void)params;
    json jRet;

    auto voltage = api_->GetAxisVoltage();
    for (const auto& v : voltage) {
        jRet["voltage"].push_back(Double2Json(v));
    }

    auto current = api_->GetAxisCurrent();
    for (const auto& v : current) {
        jRet["current"].push_back(Double2Json(v));
    }

    auto temperature = api_->GetAxisTemperature();
    for (const auto& v : temperature) {
        jRet["temperature"].push_back(Double2Json(v));
    }

    auto torque = api_->GetAxisTorque();
    for (const auto& v : torque) {
        jRet["torque"].push_back(Double2Json(v));
    }

    return jRet;
}

std::pair<bool, nlohmann::json> SdkArm::PubArmInfo()
{
    auto jRet = GetArmInfo(json());

    return {!jRet.empty(), GetArmInfo(json())};
}

nlohmann::json SdkArm::SetJointSpeedLimit(const nlohmann::json& params)
{
    json jRet;
    try {
        std::vector<double> range;
        for (size_t i = 0; i < params.size(); i++) {
            range.push_back(Json2Double(params.at(i)));
        }
        auto ret = api_->SetJointSpeedLimit(range);
        jRet = ErrState2json(ret);
    } catch (std::exception&) {
        jRet = ErrState2json(RetState::parseErr);
    }
    return jRet;
}
nlohmann::json SdkArm::GetJointSpeedLimit(const nlohmann::json& params)
{
    (void)params;
    json jRet;
    try {
        auto ret = api_->GetJointSpeedLimit();
        if (ret.first == RetState::ok) {
            for (const auto& v : ret.second) {
                jRet.push_back(Double2Json(v));
            }
        }
    } catch (std::exception&) {
        jRet = ErrState2json(RetState::parseErr);
    }
    return jRet;
}

nlohmann::json SdkArm::SetJointAccLimit(const nlohmann::json& params)
{
    json jRet;
    try {
        std::vector<double> range;
        for (size_t i = 0; i < params.size(); i++) {
            range.push_back(Json2Double(params.at(i)));
        }
        auto ret = api_->SetJointAccLimit(range);
        jRet = ErrState2json(ret);
    } catch (std::exception&) {
        jRet = ErrState2json(RetState::parseErr);
    }
    return jRet;
}
nlohmann::json SdkArm::GetJointAccLimit(const nlohmann::json& params)
{
    (void)params;
    json jRet;
    try {
        auto ret = api_->GetJointAccLimit();
        if (ret.first == RetState::ok) {
            for (const auto& v : ret.second) {
                jRet.push_back(Double2Json(v));
            }
        }
    } catch (std::exception&) {
        jRet = ErrState2json(RetState::parseErr);
    }
    return jRet;
}

nlohmann::json SdkArm::SetDragMode(const nlohmann::json& params)
{
    json jRet;
    try {
        auto ret = api_->SetDragMode(params);
        jRet = ErrState2json(ret);
    } catch (std::exception&) {
        jRet = ErrState2json(RetState::parseErr);
    }
    return jRet;
}
nlohmann::json SdkArm::GetDragMode(const nlohmann::json& params)
{
    (void)params;
    json jRet;
    try {
        auto ret = api_->GetDragMode();
        if (ret.first == RetState::ok) {
            jRet = ret.second;
        } else {
            jRet = ErrState2json(ret.first);
        }
    } catch (std::exception&) {
        jRet = ErrState2json(RetState::parseErr);
    }
    return jRet;
}

nlohmann::json SdkArm::SetRapidrate(const nlohmann::json& params)
{
    json jRet;
    try {
        auto ret = api_->SetRapidrate(Json2Double(params));
        jRet = ErrState2json(ret);
    } catch (std::exception&) {
        jRet = ErrState2json(RetState::parseErr);
    }
    return jRet;
}
nlohmann::json SdkArm::GetRapidrate(const nlohmann::json& params)
{
    (void)params;
    json jRet;
    try {
        auto ret = api_->GetRapidrate();
        if (ret.first == RetState::ok) {
            jRet = Double2Json(ret.second);
        } else {
            jRet = ErrState2json(ret.first);
        }
    } catch (std::exception&) {
        jRet = ErrState2json(RetState::parseErr);
    }
    return jRet;
}

nlohmann::json SdkArm::SetReset(const nlohmann::json& params)
{
    json jRet;
    try {
        auto ret = api_->SetReset(params);
        jRet = ErrState2json(ret);
    } catch (std::exception&) {
        jRet = ErrState2json(RetState::parseErr);
    }
    return jRet;
}
nlohmann::json SdkArm::GetReset(const nlohmann::json& params)
{
    (void)params;
    json jRet;
    try {
        auto ret = api_->GetReset();
        if (ret.first == RetState::ok) {
            jRet = ret.second;
        } else {
            jRet = ErrState2json(ret.first);
        }
    } catch (std::exception&) {
        jRet = ErrState2json(RetState::parseErr);
    }
    return jRet;
}

nlohmann::json SdkArm::SetGripper(const nlohmann::json& params)
{
    json jRet;
    try {
        auto state = params.at("state");
        auto speed = Json2Double(params.at("speed"));
        auto ret = api_->SetGripper(state, speed);
        jRet = ErrState2json(ret);
    } catch (std::exception&) {
        jRet = ErrState2json(RetState::parseErr);
    }
    return jRet;
}
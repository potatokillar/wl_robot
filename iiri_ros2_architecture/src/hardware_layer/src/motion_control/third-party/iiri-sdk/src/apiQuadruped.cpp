#include "apiQuadruped.hpp"

#include <iostream>

using namespace nlohmann;
using namespace std;

namespace iiri::qr
{
ApiQuadruped::ApiQuadruped(std::shared_ptr<SdkProtocol> rpcCli) { rpcCli_ = rpcCli; }

Result<nlohmann::json> ApiQuadruped::Call(const std::string &method, const nlohmann::json &params)
{
    return rpcCli_->Call("quadruped", method, params);
}

/**
 * @description: 设置机器人线速度
 * @param x x方向，m/s
 * @param y y方向，m/s
 * @param z z方向，m/s
 * @return {*}
 */
RetState ApiQuadruped::SetLinearVelocity(double x, double y, double z)
{
    z = 0;  // 四足无z方向速度
    nlohmann::json param = nlohmann::json::array({Double2Json(x), Double2Json(y), Double2Json(z)});
    auto ret = Call("SetLinearVelocity", param);
    if (ret.first == RetState::ok) {
        // sdk服务器有正确返回，则检查服务器返回值，查看指令是否正确执行
        return Json2errState(ret.second);
    }
    // 服务器未返回，返回未返回原因，超时或者解析错误
    return ret.first;
}

/**
 * @description: 获取机器人线速度
 * @param  &x
 * @param  &y
 * @param  &z
 * @return {*}
 */
RetState ApiQuadruped::GetLinearVelocity(double &x, double &y, double &z)
{
    nlohmann::json param;

    auto ret = Call("GetLinearVelocity", param);
    if (ret.first == RetState::ok) {
        try {
            x = Json2Double(ret.second.at(0));
            y = Json2Double(ret.second.at(1));
            z = Json2Double(ret.second.at(2));
        } catch (const std::exception &e) {
            return RetState::parseErr;
        }
    }
    return ret.first;
}

/**
 * @description: 获取速度的信息，此处只有上下限。该API可能变化，但是对外API不会变
 * @param &x
 * @param &y
 * @param &z
 * @return {}
 */
RetState ApiQuadruped::GetLinearVelocityInfo(ValueRange<double> &x, ValueRange<double> &y, ValueRange<double> &z)
{
    nlohmann::json param;

    auto ret = Call("GetLinearVelocityInfo", param);
    if (ret.first == RetState::ok) {
        try {
            x.min = Json2Double(ret.second.at("range").at("x").at(0));
            x.max = Json2Double(ret.second.at("range").at("x").at(1));
            y.min = Json2Double(ret.second.at("range").at("y").at(0));
            y.max = Json2Double(ret.second.at("range").at("y").at(1));
            z.min = Json2Double(ret.second.at("range").at("z").at(0));
            z.max = Json2Double(ret.second.at("range").at("z").at(1));
        } catch (const std::exception &) {
            return RetState::parseErr;
        }
    }
    return RetState::ok;
}

/**
 * @description: 订阅和回调
 * @param func
 * @return {}
 */
RetState ApiQuadruped::SubscribeLinearVelocity(std::function<void(const LinearVelocityData &)> func)
{
    pubLinearVelocity_ = func;
    return rpcCli_->SetSubscribe("quadruped", "PubLinearVelocity", [this](const nlohmann::json &ret) { this->RxPubLinearVelocity(ret); });
}
void ApiQuadruped::RxPubLinearVelocity(const nlohmann::json &ret)
{
    LinearVelocityData info;
    try {
        info.x = ret.at(0);
        info.y = ret.at(1);
        info.z = ret.at(2);
    } catch (const std::exception &e) {
        return;
    }

    if (pubLinearVelocity_) {
        pubLinearVelocity_(info);
    }
}

/**
 * @description: 设置机器人角速度
 * @param  x x方向，rad/s
 * @param  y y方向，rad/s
 * @param  z z方向，rad/s
 * @return {*}
 */
RetState ApiQuadruped::SetAngularVelocity(double x, double y, double z)
{
    x = 0;
    y = 0;  // 四足无x/y方向的角速度

    nlohmann::json param = {Double2Json(x), Double2Json(y), Double2Json(z)};

    auto ret = Call("SetAngularVelocity", param);
    if (ret.first == RetState::ok) {
        return Json2errState(ret.second);
    }
    return ret.first;
}

/**
 * @description: 获取机器人角速度
 * @param &x
 * @param &y
 * @param &z
 * @return {*}
 */
RetState ApiQuadruped::GetAngularVelocity(double &x, double &y, double &z)
{
    nlohmann::json param;

    auto ret = Call("GetAngularVelocity", param);
    if (ret.first == RetState::ok) {
        try {
            x = Json2Double(ret.second.at(0));
            y = Json2Double(ret.second.at(1));
            z = Json2Double(ret.second.at(2));

        } catch (const std::exception &e) {
            return RetState::parseErr;
        }
    }
    return ret.first;
}

/**
 * @description: 获取角速度的信息，此处只有上下限。该API可能变化，但是对外API不会变
 * @param &x
 * @param &y
 * @param &z
 * @return {}
 */
RetState ApiQuadruped::GetAngularVelocityInfo(ValueRange<double> &x, ValueRange<double> &y, ValueRange<double> &z)
{
    nlohmann::json param;

    auto ret = Call("GetAngularVelocityInfo", param);
    if (ret.first == RetState::ok) {
        try {
            x.min = Json2Double(ret.second.at("range").at("x").at(0));
            x.max = Json2Double(ret.second.at("range").at("x").at(1));
            y.min = Json2Double(ret.second.at("range").at("y").at(0));
            y.max = Json2Double(ret.second.at("range").at("y").at(1));
            z.min = Json2Double(ret.second.at("range").at("z").at(0));
            z.max = Json2Double(ret.second.at("range").at("z").at(1));
        } catch (const std::exception &) {
            return RetState::parseErr;
        }
    }
    return ret.first;
}

/**
 * @description: 订阅和回调
 * @param func
 * @return {}
 */
RetState ApiQuadruped::SubscribeAngularVelocity(std::function<void(const AngularVelocityData &)> func)
{
    pubAngularVelocity_ = func;
    return rpcCli_->SetSubscribe("quadruped", "PubAngularVelocity", [this](const nlohmann::json &ret) { this->RxPubAngularVelocity(ret); });
}
void ApiQuadruped::RxPubAngularVelocity(const nlohmann::json &ret)
{
    AngularVelocityData info;
    try {
        info.x = ret.at(0);
        info.y = ret.at(1);
        info.z = ret.at(2);
    } catch (const std::exception &e) {
        return;
    }

    if (pubAngularVelocity_) {
        pubAngularVelocity_(info);
    }
}

/**
 * @description: 设置姿态
 * @param roll
 * @param pitch
 * @param yaw
 * @return {}
 */
RetState ApiQuadruped::SetPose(double roll, double pitch, double yaw)
{
    nlohmann::json param = nlohmann::json::array({Double2Json(roll), Double2Json(pitch), Double2Json(yaw)});

    auto ret = Call("SetPose", param);
    if (ret.first == RetState::ok) {
        return Json2errState(ret.second);
    }

    return ret.first;
}
/**
 * @description: 获取姿态
 * @param &row
 * @param &pitch
 * @param &yaw
 * @return {}
 */
RetState ApiQuadruped::GetPose(double &roll, double &pitch, double &yaw)
{
    nlohmann::json param;

    auto ret = Call("GetPose", param);
    if (ret.first == RetState::ok) {
        try {
            roll = Json2Double(ret.second.at(0));
            pitch = Json2Double(ret.second.at(1));
            yaw = Json2Double(ret.second.at(2));

        } catch (const std::exception &e) {
            return RetState::parseErr;
        }
    }
    return ret.first;
}

RetState ApiQuadruped::GetPoseInfo(ValueRange<double> &roll, ValueRange<double> &pitch, ValueRange<double> &yaw)
{
    nlohmann::json param;

    auto ret = Call("GetPoseInfo", param);
    if (ret.first == RetState::ok) {
        try {
            roll.min = Json2Double(ret.second.at("range").at("roll").at(0));
            roll.max = Json2Double(ret.second.at("range").at("roll").at(1));
            pitch.min = Json2Double(ret.second.at("range").at("pitch").at(0));
            pitch.max = Json2Double(ret.second.at("range").at("pitch").at(1));
            yaw.min = Json2Double(ret.second.at("range").at("yaw").at(0));
            yaw.max = Json2Double(ret.second.at("range").at("yaw").at(1));
        } catch (const std::exception &) {
            return RetState::parseErr;
        }
    }
    return ret.first;
}

/**
 * @description: 订阅和回调
 * @param func
 * @return {}
 */
RetState ApiQuadruped::SubscribePose(std::function<void(const PoseData &)> func)
{
    pubPose_ = func;
    return rpcCli_->SetSubscribe("quadruped", "PubPose", [this](const nlohmann::json &ret) { this->RxPubPose(ret); });
}
void ApiQuadruped::RxPubPose(const nlohmann::json &ret)
{
    PoseData info;
    try {
        info.roll = ret.at(0);
        info.yaw = ret.at(1);
        info.pitch = ret.at(2);
    } catch (const std::exception &e) {
        return;
    }

    if (pubPose_) {
        pubPose_(info);
    }
}

/**
 * @description: 设置高度相关
 * @param set
 * @return {}
 */
RetState ApiQuadruped::SetHeight(double set)
{
    nlohmann::json param = Double2Json(set);
    auto ret = Call("SetHeight", param);
    if (ret.first == RetState::ok) {
        return Json2errState(ret.second);
    }
    return ret.first;
}
RetState ApiQuadruped::GetHeight(double &height)
{
    nlohmann::json param;
    auto ret = Call("GetHeight", param);
    if (ret.first == RetState::ok) {
        height = Json2Double(ret.second);
    }
    return ret.first;
}
RetState ApiQuadruped::GetHeightInfo(ValueRange<double> &height)
{
    nlohmann::json param;

    auto ret = Call("GetHeightInfo", param);
    if (ret.first == RetState::ok) {
        try {
            height.min = Json2Double(ret.second.at("range").at(0));
            height.max = Json2Double(ret.second.at("range").at(1));
        } catch (const std::exception &) {
            return RetState::parseErr;
        }
    }
    return ret.first;
}

/**
 * @description: 订阅和回调
 * @param func
 * @return {}
 */
RetState ApiQuadruped::SubscribeHeight(std::function<void(double)> func)
{
    pubHeight_ = func;
    return rpcCli_->SetSubscribe("quadruped", "PubHeight", [this](const nlohmann::json &ret) { this->RxPubHeight(ret); });
}
void ApiQuadruped::RxPubHeight(const nlohmann::json &ret)
{
    double info;
    try {
        info = ret;
    } catch (const std::exception &e) {
        return;
    }

    if (pubHeight_) {
        pubHeight_(info);
    }
}

/**
 * @description: 设置运行状态
 * @param sta
 * @return {}
 */
RetState ApiQuadruped::SetRunState(RunState sta)
{
    nlohmann::json param;
    if (sta == RunState::lie) {
        param = "lie";
    } else if (sta == RunState::stand) {
        param = "stand";
    } else if (sta == RunState::walk) {
        param = "walk";
    } else {
        param = "null";
    }

    auto ret = Call("SetRunState", param);
    if (ret.first == RetState::ok) {
        return Json2errState(ret.second);
    }
    return ret.first;
}
RetState ApiQuadruped::GetRunState(RunState &sta)
{
    nlohmann::json param;

    auto ret = Call("GetRunState", param);
    if (ret.first == RetState::ok) {
        try {
            if (ret.second == "lie") {
                sta = RunState::lie;
            } else if (ret.second == "stand") {
                sta = RunState::stand;
            } else if (ret.second == "walk") {
                sta = RunState::walk;
            } else if (ret.second == "switching") {
                sta = RunState::switching;
            } else {
                return RetState::noSupport;
            }

        } catch (const std::exception &e) {
            return RetState::parseErr;
        }
    }
    return ret.first;
}
RetState ApiQuadruped::GetRunStateInfo(std::vector<std::string> &runSta)
{
    nlohmann::json param;

    auto ret = Call("GetRunStateInfo", param);
    if (ret.first == RetState::ok) {
        try {
            for (auto &var : ret.second.at("restrict")) {
                runSta.push_back(var);
            }
        } catch (const std::exception &) {
            return RetState::parseErr;
        }
    }
    return ret.first;
}

/**
 * @description: 订阅和回调
 * @param func
 * @return {}
 */
RetState ApiQuadruped::SubscribeRunState(std::function<void(RunState)> func)
{
    pubRunState_ = func;
    return rpcCli_->SetSubscribe("quadruped", "PubRunState", [this](const nlohmann::json &ret) { this->RxPubRunState(ret); });
}
void ApiQuadruped::RxPubRunState(const nlohmann::json &ret)
{
    RunState info;
    try {
        if (ret == "lie") {
            info = RunState::lie;
        } else if (ret == "stand") {
            info = RunState::stand;
        } else if (ret == "walk") {
            info = RunState::walk;
        } else if (ret == "switching") {
            info = RunState::switching;
        } else {
            return;
        }

    } catch (const std::exception &e) {
        return;
    }

    if (pubRunState_) {
        pubRunState_(info);
    }
}

/**
 * @description: 设置步行模式
 * @param mode
 * @return {}
 */
RetState ApiQuadruped::SetWalkMode(WalkMode mode)
{
    nlohmann::json param;
    if (mode == WalkMode::aiNormal) {
        param = "aiNormal";
    } else if (mode == WalkMode::aiClimb) {
        param = "aiClimb";
    } else if (mode == WalkMode::aiFast) {
        param = "aiFast";
    } else {
        param = "null";
    }
    auto ret = Call("SetWalkMode", param);
    if (ret.first == RetState::ok) {
        return Json2errState(ret.second);
    }
    return ret.first;
}
RetState ApiQuadruped::GetWalkMode(WalkMode &mode)
{
    nlohmann::json param;

    auto ret = Call("GetWalkMode", param);
    if (ret.first == RetState::ok) {
        try {
            if (ret.second == "aiNormal") {
                mode = WalkMode::aiNormal;
            } else if (ret.second == "aiClimb") {
                mode = WalkMode::aiClimb;
            } else if (ret.second == "aiFast") {
                mode = WalkMode::aiFast;
            } else {
                return RetState::noSupport;
            }

        } catch (const std::exception &e) {
            return RetState::parseErr;
        }
    }
    return ret.first;
}
RetState ApiQuadruped::GetWalkModeInfo(std::vector<std::string> &runMode)
{
    nlohmann::json param;

    auto ret = Call("GetWalkModeInfo", param);
    if (ret.first == RetState::ok) {
        try {
            for (auto &var : ret.second.at("restrict")) {
                runMode.push_back(var);
            }
        } catch (const std::exception &) {
            return RetState::parseErr;
        }
    }
    return ret.first;
}

/**
 * @description: 订阅和回调
 * @param func
 * @return {}
 */
RetState ApiQuadruped::SubscribeWalkMode(std::function<void(WalkMode)> func)
{
    pubWalkMode_ = func;
    return rpcCli_->SetSubscribe("quadruped", "PubWalkMode", [this](const nlohmann::json &ret) { this->RxPubWalkMode(ret); });
}
void ApiQuadruped::RxPubWalkMode(const nlohmann::json &ret)
{
    WalkMode info;
    try {
        if (ret == "aiNormal") {
            info = WalkMode::aiNormal;
        } else if (ret == "aiClimb") {
            info = WalkMode::aiClimb;
        } else if (ret == "aiFast") {
            info = WalkMode::aiFast;
        } else {
            return;
        }
    } catch (const std::exception &e) {
        return;
    }

    if (pubWalkMode_) {
        pubWalkMode_(info);
    }
}

/**
 * @description: 设置机器人状态和控制机器人
 * @param &sta
 * @return {}
 */
RetState ApiQuadruped::GetRobotState(RobotState &sta)
{
    nlohmann::json param;

    auto ret = Call("GetRobotState", param);
    if (ret.first == RetState::ok) {
        try {
            if (ret.second == "running") {
                sta = RobotState::running;
            } else if (ret.second == "standby") {
                sta = RobotState::standby;
            } else if (ret.second == "error") {
                sta = RobotState::error;
            } else if (ret.second == "emgstop") {
                sta = RobotState::emgstop;
            } else {
                return RetState::noSupport;
            }

        } catch (const std::exception &e) {
            return RetState::parseErr;
        }
    }
    return ret.first;
}
RetState ApiQuadruped::SetRobotCtrl(RobotCtrl sta)
{
    nlohmann::json param;
    if (sta == RobotCtrl::start) {
        param = "start";
    } else if (sta == RobotCtrl::stop) {
        param = "stop";
    } else if (sta == RobotCtrl::enterEmgstop) {
        param = "enterEmgstop";
    } else if (sta == RobotCtrl::exitEmgstop) {
        param = "exitEmgstop";
    } else {
        param = "hold";
    }
    auto ret = Call("SetRobotCtrl", param);
    if (ret.first == RetState::ok) {
        return Json2errState(ret.second);
    }
    return ret.first;
}
RetState ApiQuadruped::GetRobotCtrlInfo(std::vector<std::string> &motor)
{
    nlohmann::json param;

    auto ret = Call("GetRobotCtrlInfo", param);
    if (ret.first == RetState::ok) {
        try {
            for (auto &var : ret.second.at("restrict")) {
                motor.push_back(var);
            }
        } catch (const std::exception &) {
            return RetState::parseErr;
        }
    }
    return ret.first;
}
RetState ApiQuadruped::SubscribetRobotState(std::function<void(RobotState)> func)
{
    pubRobotState_ = func;
    return rpcCli_->SetSubscribe("quadruped", "PubRobotState", [this](const nlohmann::json &ret) { this->RxPubRobotState(ret); });
}
void ApiQuadruped::RxPubRobotState(const nlohmann::json &ret)
{
    RobotState sta;
    try {
        if (ret == "running") {
            sta = RobotState::running;
        } else if (ret == "standby") {
            sta = RobotState::standby;
        } else if (ret == "error") {
            sta = RobotState::error;
        } else if (ret == "emgstop") {
            sta = RobotState::emgstop;
        } else {
            return;
        }

    } catch (const std::exception &e) {
        return;
    }

    if (pubRobotState_) {
        pubRobotState_(sta);
    }
}

RetState ApiQuadruped::SetHoldFlag(bool set)
{
    nlohmann::json param;
    param = set;
    auto ret = Call("SetHoldFlag", param);
    if (ret.first == RetState::ok) {
        return Json2errState(ret.second);
    }
    return ret.first;
}
RetState ApiQuadruped::GetHoldFlag(bool &set)
{
    nlohmann::json param;

    auto ret = Call("GetHoldFlag", param);
    if (ret.first == RetState::ok) {
        try {
            set = ret.second;
        } catch (const std::exception &e) {
            return RetState::parseErr;
        }
    }
    return ret.first;
}

RetState ApiQuadruped::SetDance(const std::string &name, uint32_t ms)
{
    nlohmann::json param = name;
    auto ret = rpcCli_->Call("quadruped", "SetDance", param, ms);
    if (ret.first == RetState::ok) {
        return Json2errState(ret.second);
    }
    return ret.first;
}

/**
 * @description: 设置高度相关
 * @param set
 * @return {}
 */
RetState ApiQuadruped::SetLoadMass(double set)
{
    nlohmann::json param = Double2Json(set);
    auto ret = Call("SetLoadMass", param);
    if (ret.first == RetState::ok) {
        return Json2errState(ret.second);
    }
    return ret.first;
}
RetState ApiQuadruped::GetLoadMass(double &mass)
{
    nlohmann::json param;
    auto ret = Call("GetLoadMass", param);
    if (ret.first == RetState::ok) {
        mass = Json2Double(ret.second);
    }
    return ret.first;
}
RetState ApiQuadruped::GetLoadMassInfo(ValueRange<double> &height)
{
    nlohmann::json param;

    auto ret = Call("GetLoadMassInfo", param);
    if (ret.first == RetState::ok) {
        try {
            height.min = Json2Double(ret.second.at("range").at(0));
            height.max = Json2Double(ret.second.at("range").at(1));
        } catch (const std::exception &) {
            return RetState::parseErr;
        }
    }
    return ret.first;
}

Result<ImuRet> ApiQuadruped::GetImuData()
{
    Result<ImuRet> retData;
    nlohmann::json param;
    auto ret = rpcCli_->Call("quadruped", "GetImuData", param);
    if (ret.first == RetState::ok) {
        try {
            if ((ret.second.is_string()) && (ret.second == "noUpdate")) {
                retData.first = RetState::noUpdate;
            } else {
                for (int i = 0; i < 3; i++) {
                    retData.second.gyro[i] = Json2Double(ret.second["gyro"][i]);
                    retData.second.acc[i] = Json2Double(ret.second["acc"][i]);
                    retData.second.ang[i] = Json2Double(ret.second["ang"][i]);
                    retData.second.quat[i] = Json2Double(ret.second["quat"][i]);
                }
                retData.second.quat[3] = Json2Double(ret.second["quat"][3]);
                retData.first = ret.first;
            }
        } catch (std::exception &) {
            retData.first = RetState::parseErr;
        }
    }
    return retData;
}

RetState ApiQuadruped::SubscribImuData(std::function<void(const ImuRet &)> func)
{
    pubImuData_ = func;
    return rpcCli_->SetSubscribe("quadruped", "PubImuData", [this](const nlohmann::json &ret) { this->RxPubImuData(ret); });
}

/**
 * @description:
 * @param &ret 该数据必定是有效数据
 * @return {}
 */
void ApiQuadruped::RxPubImuData(const nlohmann::json &ret)
{
    ImuRet info;
    try {
        for (int i = 0; i < 3; i++) {
            info.gyro[i] = Json2Double(ret["gyro"][i]);
            info.acc[i] = Json2Double(ret["acc"][i]);
            info.ang[i] = Json2Double(ret["ang"][i]);
            info.quat[i] = Json2Double(ret["quat"][i]);
        }
        info.quat[3] = Json2Double(ret["quat"][3]);
    } catch (std::exception &) {
        return;
    }

    if (pubImuData_) {
        pubImuData_(info);
    }
}

RetState ApiQuadruped::SetMotorCmd(const MotorCmd &cmd)
{
    nlohmann::json param;
    vector<string> legName = {"legFR", "legFL", "legRR", "legRL"};
    for (int i = 0; i < 4; i++) {
        param[legName[i]]["abad"].push_back(Double2Json(cmd.leg[i][0].q));
        param[legName[i]]["abad"].push_back(Double2Json(cmd.leg[i][0].qd));
        param[legName[i]]["abad"].push_back(Double2Json(cmd.leg[i][0].tau));
        param[legName[i]]["abad"].push_back(Double2Json(cmd.leg[i][0].kp));
        param[legName[i]]["abad"].push_back(Double2Json(cmd.leg[i][0].kd));

        param[legName[i]]["hip"].push_back(Double2Json(cmd.leg[i][1].q));
        param[legName[i]]["hip"].push_back(Double2Json(cmd.leg[i][1].qd));
        param[legName[i]]["hip"].push_back(Double2Json(cmd.leg[i][1].tau));
        param[legName[i]]["hip"].push_back(Double2Json(cmd.leg[i][1].kp));
        param[legName[i]]["hip"].push_back(Double2Json(cmd.leg[i][1].kd));

        param[legName[i]]["knee"].push_back(Double2Json(cmd.leg[i][2].q));
        param[legName[i]]["knee"].push_back(Double2Json(cmd.leg[i][2].qd));
        param[legName[i]]["knee"].push_back(Double2Json(cmd.leg[i][2].tau));
        param[legName[i]]["knee"].push_back(Double2Json(cmd.leg[i][2].kp));
        param[legName[i]]["knee"].push_back(Double2Json(cmd.leg[i][2].kd));
    }

    rpcCli_->QuickCall("quadruped", "SetMotorCmd", param);  // 该值不需要关心返回值

    return RetState::ok;
}

Result<MotorRet> ApiQuadruped::GetMotorData()
{
    Result<MotorRet> retData;
    nlohmann::json param;

    auto ret = rpcCli_->Call("quadruped", "GetMotorData", param);
    if (ret.first == RetState::ok) {
        try {
            if ((ret.second.is_string()) && (ret.second == "noUpdate")) {
                retData.first = RetState::noUpdate;
            } else {
                vector<string> legName = {"legFR", "legFL", "legRR", "legRL"};

                for (int i = 0; i < 4; i++) {
                    retData.second.leg[i][0].q = ret.second[legName[i]]["abad"][0];
                    retData.second.leg[i][0].qd = ret.second[legName[i]]["abad"][1];
                    retData.second.leg[i][0].tau = ret.second[legName[i]]["abad"][2];

                    retData.second.leg[i][1].q = ret.second[legName[i]]["hip"][0];
                    retData.second.leg[i][1].qd = ret.second[legName[i]]["hip"][1];
                    retData.second.leg[i][1].tau = ret.second[legName[i]]["hip"][2];

                    retData.second.leg[i][2].q = ret.second[legName[i]]["knee"][0];
                    retData.second.leg[i][2].qd = ret.second[legName[i]]["knee"][1];
                    retData.second.leg[i][2].tau = ret.second[legName[i]]["knee"][2];
                }
                retData.first = ret.first;
            }
        } catch (std::exception &) {
            retData.first = RetState::parseErr;
        }
    }
    return retData;
}

RetState ApiQuadruped::SubscribMotorData(std::function<void(const MotorRet &)> func)
{
    pubMotorData_ = func;
    return rpcCli_->SetSubscribe("quadruped", "PubMotorData", [this](const nlohmann::json &ret) { this->RxPubMotorData(ret); });
}

void ApiQuadruped::RxPubMotorData(const nlohmann::json &ret)
{
    MotorRet data;
    try {
        if ((ret.is_string()) && (ret == "noUpdate")) {
            return;
        }
        vector<string> legName = {"legFR", "legFL", "legRR", "legRL"};

        for (int i = 0; i < 4; i++) {
            data.leg[i][0].q = ret[legName[i]]["abad"][0];
            data.leg[i][0].qd = ret[legName[i]]["abad"][1];
            data.leg[i][0].tau = ret[legName[i]]["abad"][2];

            data.leg[i][1].q = ret[legName[i]]["hip"][0];
            data.leg[i][1].qd = ret[legName[i]]["hip"][1];
            data.leg[i][1].tau = ret[legName[i]]["hip"][2];

            data.leg[i][2].q = ret[legName[i]]["knee"][0];
            data.leg[i][2].qd = ret[legName[i]]["knee"][1];
            data.leg[i][2].tau = ret[legName[i]]["knee"][2];
        }
    } catch (std::exception &) {
        return;
    }

    if (pubMotorData_) {
        pubMotorData_(data);
    }
}
}  // namespace iiri::qr
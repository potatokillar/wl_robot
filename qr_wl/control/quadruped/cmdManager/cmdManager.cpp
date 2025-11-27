#include "cmdManager.hpp"

#include "baseline.hpp"

static bool DoubleIsEqual(double a, double b) { return fabs(a - b) < 0.001 ? true : false; }

CmdManager::CmdManager()
{
    userSetRange_.loadMass = qrParam.paramRange.loadMass;
    SetWalkModeRange(WalkMode::aiNormal); // 默认范围是normal的
    cmd_.loadMass = qrParam.loadMass;
    motorCmdQueue_ = std::make_shared<std::vector<CmdVal2>>();
    // 注册rpc接收
    rpcRxDeal_.emplace_back("qr::SetLinearVelocity", [this](MsgType set)
                            { return this->RxSetLinearVelocity(set); });
    rpcRxDeal_.emplace_back("qr::SetAngularVelocity", [this](MsgType set)
                            { return this->RxSetAngularVelocity(set); });
    rpcRxDeal_.emplace_back("qr::SetPose", [this](MsgType set)
                            { return this->RxSetPose(set); });
    rpcRxDeal_.emplace_back("qr::SetWalkMode", [this](MsgType set)
                            { return this->RxSetWalkMode(set); });
    rpcRxDeal_.emplace_back("qr::SetGaitType", [this](MsgType set)
                            { return this->RxSetGaitType(set); });
    rpcRxDeal_.emplace_back("qr::SetHoldFlag", [this](MsgType set)
                            { return this->RxSetHoldFlag(set); });
    rpcRxDeal_.emplace_back("qr::SetHeight", [this](MsgType set)
                            { return this->RxSetHeight(set); });
    rpcRxDeal_.emplace_back("qr::SetLoadMass", [this](MsgType set)
                            { return this->RxSetLoadMass(set); });
    rpcRxDeal_.emplace_back("qr::SetMessage", [this](MsgType set)
                            { return this->RxSetMessage(set); });
    rpcRxDeal_.emplace_back("qr::SetMotorCmdQ", [this](MsgType set)
                            { return this->RxSetMotorCmdQ(set); });
    for (auto &rpc : rpcRxDeal_)
    {
        rpc.Connect();
    }
    moveCmdLastTime_ = TimerTools::GetNowTickMs();
}

CmdManager::~CmdManager()
{
    for (auto &rpc : rpcRxDeal_)
    {
        rpc.DisConnect();
    }
}

void CmdManager::Run()
{
    for (auto &rpc : rpcRxDeal_)
    {
        rpc.Run();
    }
    // 运动指令，3秒未接受新指令，则强制为0
    if (TimerTools::GetNowTickMs() - moveCmdLastTime_ > 3000)
    {
        cmd_.linearV = Vec3<double>::Zero();
        cmd_.angularV = Vec3<double>::Zero();
        cmd_.pose = Vec3<double>::Zero();
        // LOG_DEBUG("no cmd in 3s, force to stop");
        moveCmdLastTime_ = TimerTools::GetNowTickMs();
    }
}

Vec3<double> CmdManager::GetLinear() const { return cmd_.linearV; }
Vec3<double> CmdManager::GetAngular() const { return cmd_.angularV; }
Vec3<double> CmdManager::GetPose() const { return cmd_.pose; }
bool CmdManager::GetHoldflag() const { return cmd_.holdFlag; }
double CmdManager::GetHeight() const { return cmd_.height; }
WalkMode CmdManager::GetWalkMode() const { return cmd_.mode; }
GaitType CmdManager::GetGaitType() const { return cmd_.gait; }
RunState CmdManager::GetRunState() const { return cmd_.nextState; }
double CmdManager::GetLoadMass() const { return cmd_.loadMass; }

/**
 * @description: 修改模式时，产生范围值改变，todo考虑转配置
 * @param mode
 * @return {}
 */
void CmdManager::SetWalkModeRange(WalkMode mode)
{
    if (qrParam.model.back() == QrModel::middle)
    {
        if (mode == WalkMode::normal)
        {
            userSetRange_.v.max << 0.8, 0.3, 0.0;
            userSetRange_.v.min << -0.4, -0.3, 0.0;
            userSetRange_.w.max << 0.0, 0.0, 1.0;
            userSetRange_.w.min << 0.0, 0.0, -1.0;
            userSetRange_.p.max << 0.3, 0.4, 0.4;
            userSetRange_.p.min << -0.3, -0.4, -0.4;
            userSetRange_.h.max = qrParam.paramRange.height.max;
            userSetRange_.h.min = qrParam.paramRange.height.min;
        }
        else if (mode == WalkMode::climb)
        {
            userSetRange_.v.max << 0.2, 0.2, 0.0;
            userSetRange_.v.min << -0.2, -0.2, 0.0;
            userSetRange_.w.max << 0.0, 0.0, 0.5;
            userSetRange_.w.min << 0.0, 0.0, -0.5;
            userSetRange_.p.max << 0.3, 0.4, 0.4;
            userSetRange_.p.min << -0.3, -0.4, -0.4;
            userSetRange_.h.max = qrParam.bodyHeight;
            userSetRange_.h.min = qrParam.bodyHeight;
        }
    }
    else if (qrParam.model.contains(QrModel::middleV3))
    {
        if (mode == WalkMode::normal)
        {
            userSetRange_.v.max << qrParam.paramRange.LinearVelocity.x.max, qrParam.paramRange.LinearVelocity.y.max, 0.0;
            userSetRange_.v.min << qrParam.paramRange.LinearVelocity.x.min, qrParam.paramRange.LinearVelocity.y.min, 0.0;
            userSetRange_.w.max << 0.0, 0.0, qrParam.paramRange.AngularVelocity.z.max;
            userSetRange_.w.min << 0.0, 0.0, qrParam.paramRange.AngularVelocity.z.min;
            userSetRange_.p.max << qrParam.paramRange.Pose.roll.max, qrParam.paramRange.Pose.pitch.max, qrParam.paramRange.Pose.yaw.max;
            userSetRange_.p.min << qrParam.paramRange.Pose.roll.min, qrParam.paramRange.Pose.pitch.min, qrParam.paramRange.Pose.yaw.min;
            userSetRange_.h.max = qrParam.paramRange.height.max;
            userSetRange_.h.min = qrParam.paramRange.height.min;
        }
        else if (mode == WalkMode::climb)
        {
            userSetRange_.v.max << 0.8, 0.3, 0.0;
            userSetRange_.v.min << -0.5, -0.3, 0.0;
            userSetRange_.w.max << 0.0, 0.0, 0.5;
            userSetRange_.w.min << 0.0, 0.0, -0.5;
            userSetRange_.p.max << 0.3, 0.4, 0.4;
            userSetRange_.p.min << -0.3, -0.4, -0.4;
            userSetRange_.h.max = qrParam.bodyHeight;
            userSetRange_.h.min = qrParam.bodyHeight;
        }
        else if (mode == WalkMode::light)
        {
            userSetRange_.v.max << 0.7, 0.2, 0.0;
            userSetRange_.v.min << -0.5, -0.2, 0.0;
            userSetRange_.w.max << 0.0, 0.0, 0.5;
            userSetRange_.w.min << 0.0, 0.0, -0.5;
            userSetRange_.p.max << 0.3, 0.4, 0.4;
            userSetRange_.p.min << -0.3, -0.4, -0.4;
            userSetRange_.h.max = qrParam.bodyHeight;
            userSetRange_.h.min = qrParam.bodyHeight;
        }
        else if (mode == WalkMode::aiNormal)
        {
            userSetRange_.v.max << 0.8, 0.6, 0.0;
            userSetRange_.v.min << -0.6, -0.6, 0.0;
            userSetRange_.w.max << 0.0, 0.0, 1.5;
            userSetRange_.w.min << 0.0, 0.0, -1.5;
            userSetRange_.p.max << 0.2, 0.2, 0.4;
            userSetRange_.p.min << -0.2, -0.2, -0.4;
            userSetRange_.h.max = qrParam.paramRange.height.max;
            userSetRange_.h.min = qrParam.paramRange.height.min;
        }
        else if (mode == WalkMode::aiClimb)
        {
            userSetRange_.v.max << 0.8, 0.6, 0.0;
            userSetRange_.v.min << -0.6, -0.6, 0.0;
            userSetRange_.w.max << 0.0, 0.0, 1.5;
            userSetRange_.w.min << 0.0, 0.0, -1.5;
            userSetRange_.p.max << 0.2, 0.2, 0.4;
            userSetRange_.p.min << -0.2, -0.2, -0.4;
            userSetRange_.h.max = qrParam.paramRange.height.max;
            userSetRange_.h.min = qrParam.paramRange.height.min;
        }
    }
    else if (qrParam.model.contains(QrModel::linkV2_3_w) || qrParam.model.contains(QrModel::gazebo_w))
    {
        if (mode == WalkMode::aiNormal)
        {
            userSetRange_.v.max << 0.75, 0.0, 0.0;
            userSetRange_.v.min << -0.75, -0.0, 0.0;
            userSetRange_.w.max << 0.0, 0.0, 0.7;
            userSetRange_.w.min << 0.0, 0.0, -0.7;
            userSetRange_.p.max << 0.4, 0.4, 0.4;
            userSetRange_.p.min << -0.4, -0.4, -0.4;
            userSetRange_.h.max = qrParam.paramRange.height.max;
            userSetRange_.h.min = qrParam.paramRange.height.min;
        }
        else if (mode == WalkMode::aiClimb)
        {
            userSetRange_.v.max << 0.75, 0.0, 0.0;
            userSetRange_.v.min << -0.75, -0.0, 0.0;
            userSetRange_.w.max << 0.0, 0.0, 0.7;
            userSetRange_.w.min << 0.0, 0.0, -0.7;
            userSetRange_.p.max << 0.4, 0.4, 0.4;
            userSetRange_.p.min << -0.4, -0.4, -0.4;
            userSetRange_.h.max = qrParam.paramRange.height.max;
            userSetRange_.h.min = qrParam.paramRange.height.min;
        }
    }
    else
    {
        if (mode == WalkMode::aiNormal)
        {
            userSetRange_.v.max << 1.5, 0.6, 0.0;
            userSetRange_.v.min << -1.5, -0.6, 0.0;
            userSetRange_.w.max << 0.0, 0.0, 2.0;
            userSetRange_.w.min << 0.0, 0.0, -2.0;
            userSetRange_.p.max << 0.4, 0.4, 0.4;
            userSetRange_.p.min << -0.4, -0.4, -0.4;
            userSetRange_.h.max = qrParam.paramRange.height.max;
            userSetRange_.h.min = qrParam.paramRange.height.min;
        }
        else if (mode == WalkMode::aiClimb)
        {
            userSetRange_.v.max << 1.1, 0.6, 0.0;
            userSetRange_.v.min << -1.0, -0.6, 0.0;
            userSetRange_.w.max << 0.0, 0.0, 1.4;
            userSetRange_.w.min << 0.0, 0.0, -1.4;
            userSetRange_.p.max << 0.4, 0.4, 0.4;
            userSetRange_.p.min << -0.4, -0.4, -0.4;
            userSetRange_.h.max = qrParam.paramRange.height.max;
            userSetRange_.h.min = qrParam.paramRange.height.min;
        }
        else if (mode == WalkMode::climb)
        {
            userSetRange_.v.max << 0.8, 0.4, 0.0;
            userSetRange_.v.min << -0.5, -0.4, 0.0;
            userSetRange_.w.max << 0.0, 0.0, 1.0;
            userSetRange_.w.min << 0.0, 0.0, -1.0;
            userSetRange_.p.max << 0.2, 0, 0.4;
            userSetRange_.p.min << -0.2, -0, -0.4;
            // userSetRange_.h.max = qrParam.bodyHeight;
            // userSetRange_.h.min = qrParam.bodyHeight;
            userSetRange_.h.max = qrParam.paramRange.height.max;
            userSetRange_.h.min = qrParam.paramRange.height.min;
        }
        else if (mode == WalkMode::light)
        {
            userSetRange_.v.max << 0.8, 0.3, 0.0;
            userSetRange_.v.min << -0.5, -0.3, 0.0;
            userSetRange_.w.max << 0.0, 0.0, 1.0;
            userSetRange_.w.min << 0.0, 0.0, -1.0;
            userSetRange_.p.max << 0.2, 0, 0.4;
            userSetRange_.p.min << -0.2, -0, -0.4;
            // userSetRange_.h.max = qrParam.bodyHeight;
            // userSetRange_.h.min = qrParam.bodyHeight;
            userSetRange_.h.max = qrParam.paramRange.height.max;
            userSetRange_.h.min = qrParam.paramRange.height.min;
        }
        else if (mode == WalkMode::normal)
        {
            userSetRange_.v.max << 1.5, 0.8, 0.0;
            userSetRange_.v.min << -0.5, -0.8, 0.0;
            userSetRange_.w.max << 0.0, 0.0, 2.0;
            userSetRange_.w.min << 0.0, 0.0, -2.0;
            userSetRange_.p.max << 0.5, 0.4, 0.4;
            userSetRange_.p.min << -0.5, -0.4, -0.4;
            userSetRange_.h.max = qrParam.paramRange.height.max;
            userSetRange_.h.min = qrParam.paramRange.height.min;
        }
    }
    // 范围修改后，必须重新刷新之前的限定值，超过限定的值得修改
    auto &user = cmd_;
    RxSetLinearVelocity(user.linearV);
    RxSetAngularVelocity(user.angularV);
    RxSetPose(user.pose);
    RxSetHeight(user.height - qrParam.bodyHeight); // 设置的是偏差值
    PublishParamRange();                           // 改变后发布参数范围
}

/**
 * @description: 发布范围改变消息
 * @return {}
 */
void CmdManager::PublishParamRange()
{
    ParamRange ret;

    ret.LinearVelocity.x.min = userSetRange_.v.min(0);
    ret.LinearVelocity.x.max = userSetRange_.v.max(0);
    ret.LinearVelocity.y.min = userSetRange_.v.min(1);
    ret.LinearVelocity.y.max = userSetRange_.v.max(1);
    ret.LinearVelocity.z.min = userSetRange_.v.min(2);
    ret.LinearVelocity.z.max = userSetRange_.v.max(2);

    ret.AngularVelocity.x.min = userSetRange_.w.min(0);
    ret.AngularVelocity.x.max = userSetRange_.w.max(0);
    ret.AngularVelocity.y.min = userSetRange_.w.min(1);
    ret.AngularVelocity.y.max = userSetRange_.w.max(1);
    ret.AngularVelocity.z.min = userSetRange_.w.min(2);
    ret.AngularVelocity.z.max = userSetRange_.w.max(2);

    ret.Pose.roll.min = userSetRange_.p.min(0);
    ret.Pose.roll.max = userSetRange_.p.max(0);
    ret.Pose.pitch.min = userSetRange_.p.min(1);
    ret.Pose.pitch.max = userSetRange_.p.max(1);
    ret.Pose.yaw.min = userSetRange_.p.min(2);
    ret.Pose.yaw.max = userSetRange_.p.max(2);

    ret.height.min = userSetRange_.h.min;
    ret.height.max = userSetRange_.h.max;

    ret.loadMass.min = userSetRange_.loadMass.min;
    ret.loadMass.max = userSetRange_.loadMass.max;

    MsgTrySend("qr::GetParamRange", ret);
}

///////////////////新接收函数////////////////////////////////
MsgType CmdManager::RxSetLinearVelocity(const MsgType &set)
{
    auto v = set.GetType<Vec3<double>>();
    RetState error = RetState::ok;
    auto &linearV = cmd_.linearV;
    auto &vRange = userSetRange_.v;
    for (int i = 0; i < 3; i++)
    {
        linearV(i) = mathFun_.SetValueRange(v(i), vRange.min(i), vRange.max(i), "linearV:" + std::to_string(i));
        if (DoubleIsEqual(linearV(i), v(i)) == false)
        {
            error = RetState::outRange; // 返回错误码
        }
    }

    moveCmdLastTime_ = TimerTools::GetNowTickMs();
    return MsgType(error);
}
MsgType CmdManager::RxSetAngularVelocity(const MsgType &set)
{
    auto v = set.GetType<Vec3<double>>();
    RetState error = RetState::ok;
    auto &angularV = cmd_.angularV;
    auto &wRange = userSetRange_.w;

    for (int i = 0; i < 3; i++)
    {
        angularV(i) = mathFun_.SetValueRange(v(i), wRange.min(i), wRange.max(i), "angularV:" + std::to_string(i));
        if (DoubleIsEqual(angularV(i), v(i)) == false)
        {
            error = RetState::outRange; // 返回错误码
        }
    }

    moveCmdLastTime_ = TimerTools::GetNowTickMs();
    return MsgType(error);
}
MsgType CmdManager::RxSetPose(const MsgType &set)
{
    auto p = set.GetType<Vec3<double>>();
    RetState error = RetState::ok;
    auto &pose = cmd_.pose;
    auto &pRange = userSetRange_.p;

    for (int i = 0; i < 3; i++)
    {
        pose(i) = mathFun_.SetValueRange(p(i), pRange.min(i), pRange.max(i), "pose:" + std::to_string(i));
        if (DoubleIsEqual(pose(i), p(i)) == false)
        {
            error = RetState::outRange; // 返回错误码
        }
    }

    moveCmdLastTime_ = TimerTools::GetNowTickMs();
    return MsgType(error);
}
MsgType CmdManager::RxSetHeight(const MsgType &set)
{
    auto high = set.GetType<double>();
    RetState error = RetState::ok;
    auto &height = cmd_.height;
    auto &hRange = userSetRange_.h;
    auto actul = high + qrParam.bodyHeight;

    height = mathFun_.SetValueRange(actul, hRange.min, hRange.max, "height:");
    if (DoubleIsEqual(height, actul) == false)
    {
        error = RetState::outRange; // 返回错误码
    }
    else
    {
        // LOG_INFO("SetHeight:{}", height);
    }

    return MsgType(error);
}
MsgType CmdManager::RxSetWalkMode(const MsgType &set)
{
    auto &mode = cmd_.mode;

    mode = set.GetType<WalkMode>();
    if (mode == WalkMode::aiClimb)
    {
        LOG_INFO("enter aiClimb mode");
    }
    else if (mode == WalkMode::aiNormal)
    {
        LOG_INFO("enter aiNormal mode");
    }
    else if (mode == WalkMode::normal)
    {
        LOG_INFO("enter Normal mode");
    }

    SetWalkModeRange(mode);

    return RetState::ok;
}
MsgType CmdManager::RxSetGaitType(const MsgType &set)
{
    cmd_.gait = set.GetType<GaitType>();
    return RetState::ok;
}
MsgType CmdManager::RxSetHoldFlag(const MsgType &set)
{
    cmd_.holdFlag = set.GetType<bool>();
    return MsgType(RetState::ok);
}
MsgType CmdManager::RxSetMessage(const MsgType &set)
{
    auto msg = set.GetType<std::string>();
    return true;
}

MsgType CmdManager::RxSetLoadMass(const MsgType &set)
{
    double mass = set.GetType<double>();
    RetState ret = RetState::ok;
    auto &loadMass = cmd_.loadMass;
    auto &massRange = userSetRange_.loadMass;

    loadMass = mathFun_.SetValueRange(mass, massRange.min, massRange.max, "loadMass:");
    if (DoubleIsEqual(loadMass, mass) == false)
    {
        ret = RetState::outRange; // 返回错误码
    }
    // 设置成功同时写入数据库
    CppSqlite sql;
    if (sql.SyncWrite("quadruped", "loadMass", "double", std::to_string(loadMass)) == false)
    {
        LOG_ERROR("loadMass sql update failed");
        ret = RetState::error;
    }
    if (ret == RetState::ok)
    {
        LOG_INFO("SetLoadMass:{:.2f}", loadMass);
    }
    return ret;
}

MsgType CmdManager::RxSetMotorCmdQ(const MsgType &cmd)
{
    motorCmdQueue_ = cmd.GetType<std::shared_ptr<std::vector<CmdVal2>>>();
    return RetState::ok;
}

const std::shared_ptr<std::vector<CmdVal2>> CmdManager::GetMotorCmdQueue() const { return motorCmdQueue_; }

void CmdManager::ClearMotorCmdQueue() { motorCmdQueue_->clear(); }

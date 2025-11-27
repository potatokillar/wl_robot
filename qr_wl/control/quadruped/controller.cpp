
#include "controller.hpp"

#include "baseline.hpp"
#include "ctrlNode.hpp"
#include "qrTask.hpp"

using namespace std;

ControllerCmd::ControllerCmd()
{
    nextState = AlgoState::lie;
    mode = WalkMode::aiNormal;
    gait = GaitType::tort;
    height = GetQrParam().bodyHeight;
    linearV.setZero();
    angularV.setZero();
    pose.setZero();
    holdFlag = true;
};
ControllerData::ControllerData()
{
    state = AlgoState::lie;
    mode = WalkMode::aiNormal;
    gait = GaitType::tort;
    height = GetQrParam().bodyHeight;
    linearV.setZero();
    angularV.setZero();
    pose.setZero();
    holdFlag = true;
}

void Controller::Start(const BootArgs &args)
{
    if (GetQrParam().model.back() != QrModel::unused) {
        QrBootArgs qrArgs;
        qrArgs.dt = 0.02;
        qrArgs.algoModel = GetQrParam().algoModel;
        CtrlNode::GetInstance().Start(args, qrArgs);
        isRun_ = true;
    }
}
void Controller::Stop()
{
    CtrlNode::GetInstance().Stop();
    // 复位数据
    ctrlData_ = {};
    isRun_ = false;
}

/**
 * @description: 算法是否已经起来
 * @return {}
 */
bool Controller::IsRun() { return isRun_; }

Controller::Controller() { GetParamRange(); }

/**
 * @description: 比例值转实际值
 * @param v
 * @param range
 * @return {}
 */
double Controller::Ratio2Value(double v, ValueRange<double> range)
{
    v = std::max(std::min(v, 1.0), -1.0);  // 限制比例值

    // 范围有正有负，正负系数独立计算
    if ((range.min <= 0) && (range.max >= 0)) {
        return (v >= 0) ? (range.max * v) : (range.min * (-v));
    }

    // 范围永远为正值，那负值系数无意义
    if ((range.min >= 0) && (range.max >= 0)) {
        return (v >= 0) ? (range.min + (range.max - range.min) * v) : 0;
    }

    // 范围永远为负值，那正值系数无意义
    if ((range.min <= 0) && (range.max <= 0)) {
        return (v <= 0) ? (range.min + (range.max - range.min) * (-v)) : 0;
    }
    LOG_ERROR("Ratio2Value reach unexpected!");
    return 0;  // 决不会运行到这里
}

/**
 * @description: 实际值转比例值
 * @param r
 * @param range
 * @return {}
 */
double Controller::Value2Ratio(double r, ValueRange<double> range)
{
    if (r < range.min) {
        LOG_ERROR("Value2Ratio < range min! return min");
        return range.min;
    }
    if (r > range.max) {
        LOG_ERROR("Value2Ratio > range max! return max");
        return range.max;
    }

    // 范围有正有负，正负系数独立计算
    if ((range.min <= 0) && (range.max >= 0)) {
        return (r >= 0) ? (r / range.max) : (r / range.min);
    }

    // 范围永远为正值，比例一定是正值
    if ((range.min >= 0) && (range.max >= 0)) {
        return (r - range.min) / (range.max - range.min);
    }

    // 范围永远为负值，比例一定是负值
    if ((range.min <= 0) && (range.max <= 0)) {
        return (r - range.max) / (range.max - range.min);
    }

    LOG_ERROR("Value2Ratio reach unexpected!");
    return 0;  // 决不会运行到这里
}

/**
 * @description: 获取算法对外数据，todo数据类型得修改一下，控制消息类型的传染范围
 * @param data
 * @return {}
 */
bool Controller::GetCtrlData(ControllerData *data) const
{
    auto ret = MsgTryRecv<ControllerData>("qr::GetCtrlData", this);
    if (ret) {
        *data = ret.value();
        return true;
    }
    return false;
}

/**
 * @description: 设置imu数据
 * @param data
 * @return {}
 */
void Controller::SetImuData(const msg::imu_data &data) { MsgTrySend("qr::imuData", data); }
/**
 * @description: 设置motorData数据
 * @param data
 * @return {}
 */
void Controller::SetMotorData(const msg::qr::motor_ret &data) { MsgTrySend("qr::motor_ret", data); }

bool Controller::ret2flagX(msg::qr::motor_cmd *cmd)
{
    auto ret = MsgTryRecv<msg::qr::motor_cmd>("qr::motor_cmd", this);
    if (ret) {
        *cmd = ret.value();
        return true;
    }
    return false;
}

RetState Controller::SetLinearVelocity(double x, double y, double z)
{
    Vec3<double> set(x, y, z);
    RetState ret;
    // 调用成功返回实际返回值，调用失败返回timeout错误
    return RpcTrySend("qr::SetLinearVelocity", set, &ret) == true ? ret : RetState::timeout;
}

VelocityType<double> Controller::GetLinearVelocity()
{
    auto ret = MsgTryRecv<ControllerData>("qr::ControllerData", this);
    if (ret) {
        ctrlData_ = ret.value();
    }

    return {ctrlData_.linearV(0), ctrlData_.linearV(1), ctrlData_.linearV(2)};
}
/**
 * @description: 获取线速度的范围
 * todo，应该获取不同模式下的速度，而不是直接从参数获取
 * @return {}
 */
VelocityType<ValueRange<double>> Controller::GetLinearVelocityRange()
{
    const auto &vRange = GetParamRange().LinearVelocity;

    return {vRange.x, vRange.y, vRange.z};
}

/**
 * @description: 按照比例方式设置线速度
 * @param x [-1, 1]，-1是最小值，1是最大值
 * @param y
 * @param z
 * @return {}
 */
RetState Controller::SetLinearVelocityRatio(double x, double y, double z)
{
    const auto &vRange = GetParamRange().LinearVelocity;

    double vx = Ratio2Value(x, vRange.x);
    double vy = Ratio2Value(y, vRange.y);
    double vz = Ratio2Value(z, vRange.z);

    Vec3<double> linearV(vx, vy, vz);
    RetState ret;
    return RpcTrySend("qr::SetLinearVelocity", linearV, &ret) == true ? ret : RetState::timeout;
}

RetState Controller::SetAngularVelocity(double x, double y, double z)
{
    Vec3<double> angularV(x, y, z);
    RetState ret;
    return RpcTrySend("qr::SetAngularVelocity", angularV, &ret) == true ? ret : RetState::timeout;
}
RetState Controller::SetAngularVelocityRatio(double x, double y, double z)
{
    const auto &aRange = GetParamRange().AngularVelocity;

    double vx = Ratio2Value(x, aRange.x);
    double vy = Ratio2Value(y, aRange.y);
    double vz = Ratio2Value(z, aRange.z);

    Vec3<double> angularV = {vx, vy, vz};

    RetState ret;
    return RpcTrySend("qr::SetAngularVelocity", angularV, &ret) == true ? ret : RetState::timeout;
}
VelocityType<double> Controller::GetAngularVelocity()
{
    GetCtrlData();
    return {ctrlData_.angularV(0), ctrlData_.angularV(1), ctrlData_.angularV(2)};
}
VelocityType<ValueRange<double>> Controller::GetAngularVelocityRange()
{
    const auto &aRange = GetParamRange().AngularVelocity;
    return {aRange.x, aRange.y, aRange.z};
}

RetState Controller::SetPoseRatio(double roll, double pitch, double yaw)
{
    const auto &pRange = GetParamRange().Pose;

    double vRoll = Ratio2Value(roll, pRange.roll);
    double vPitch = Ratio2Value(pitch, pRange.pitch);
    double vYaw = Ratio2Value(yaw, pRange.yaw);

    Vec3<double> pose = {vRoll, vPitch, vYaw};

    RetState ret;
    return RpcTrySend("qr::SetPose", pose, &ret) == true ? ret : RetState::timeout;
}
RetState Controller::SetPose(double roll, double pitch, double yaw)
{
    Vec3<double> pose = {roll, pitch, yaw};

    RetState ret;
    return RpcTrySend("qr::SetPose", pose, &ret) == true ? ret : RetState::timeout;
}
PoseType<double> Controller::GetPose()
{
    GetCtrlData();
    return {ctrlData_.pose(0), ctrlData_.pose(1), ctrlData_.pose(2)};
}
PoseType<ValueRange<double>> Controller::GetPoseRange()
{
    const auto &pRange = GetParamRange().Pose;
    return {pRange.roll, pRange.pitch, pRange.yaw};
}

RetState Controller::SetHeightRatio(double ratio)
{
    auto range = GetParamRange().height;
    range.min -= GetQrParam().bodyHeight;
    range.max -= GetQrParam().bodyHeight;
    double vHeight = Ratio2Value(ratio, range);
#if 0
    MsgRpcSend rpc("qr::SetHeight");
    auto resp = rpc.Request(vHeight);
    if (resp) {
        return resp.value().GetType<RetState>();
    }
    return RetState::timeout;
#endif
    RetState ret;
    return RpcTrySend("qr::SetHeight", vHeight, &ret) ? ret : RetState::timeout;
}
/**
 * @description: 设置高度偏差值
 * @param height 默认高度的偏差
 * @return {}
 */
RetState Controller::SetHeight(double heightOffset)
{
#if 0
    MsgRpcSend rpc("qr::SetHeight");
    auto resp = rpc.Request(heightOffset);
    if (resp) {
        return resp.value().GetType<RetState>();
    }
    return RetState::timeout;
#endif
    RetState ret;
    return RpcTrySend("qr::SetHeight", heightOffset, &ret) ? ret : RetState::timeout;
}
/**
 * @description: 获取高度偏差值
 * @return {} 基于基准高度的上下偏差值
 */
double Controller::GetHeight()
{
    GetCtrlData();
    return ctrlData_.height - GetQrParam().bodyHeight;
}

double Controller::GetHeightRatio()
{
    GetCtrlData();
    auto range = GetParamRange().height;
    range.min -= GetQrParam().bodyHeight;
    range.max -= GetQrParam().bodyHeight;
    return Value2Ratio(ctrlData_.height - GetQrParam().bodyHeight, range);
}

/**
 * @description: 获取高度范围，这里是基于基准高度的上下范围
 * @return {}
 */
ValueRange<double> Controller::GetHeightRange()
{
    auto ret = GetParamRange().height;
    ret.min -= GetQrParam().bodyHeight;
    ret.max -= GetQrParam().bodyHeight;

    return ret;
}

RetState Controller::SetLoadMass(double mass)
{
    RetState ret;
    return RpcTrySend("qr::SetLoadMass", mass, &ret) ? ret : RetState::timeout;
}
double Controller::GetLoadMass()
{
    GetCtrlData();
    return ctrlData_.loadMass;
}

ValueRange<double> Controller::GetLoadMassRange() { return GetParamRange().loadMass; }

RetState Controller::SetRunState(AlgoState sta)
{
    RetState ret;
    bool rpcSuccess = RpcTrySend("qr::SetRunState", sta, &ret);
    if (!rpcSuccess)
    {
        LOG_ERROR("[Controller] Internal RPC 'qr::SetRunState' TIMEOUT! No response within 20ms");
        ret = RetState::timeout;
    }
    else
    {
        LOG_INFO("[Controller] Internal RPC 'qr::SetRunState' returned:{}", static_cast<int>(ret));
    }
    // SetWalkMode(WalkMode::aiNormal);
    return ret;
}
AlgoState Controller::GetRunState()
{
    GetCtrlData();
    return ctrlData_.state;
}

/**
 * @description: 获取runState的设置范围
 * 不同型号的机器人，其设置范围可能是不同的
 * @return {}
 */
std::vector<AlgoState> Controller::GetRunStateRange() { return GetQrParam().paramRange.runState; }

RetState Controller::SetWalkMode(WalkMode mode)
{
    RetState ret;
    ret = RpcTrySend("qr::SetWalkMode", mode, &ret) == true ? ret : RetState::timeout;
    return ret;
}
WalkMode Controller::GetWalkMode()
{
    GetCtrlData();
    return ctrlData_.mode;
}
std::vector<WalkMode> Controller::GetWalkModeRange() { return GetQrParam().paramRange.walkMode; }

RetState Controller::SetGaitType(GaitType gait)
{
    RetState ret;
    ret = RpcTrySend("qr::SetGaitType", gait, &ret) == true ? ret : RetState::timeout;
    return ret;
}
GaitType Controller::GetGaitType()
{
    GetCtrlData();
    return ctrlData_.gait;
}
std::vector<GaitType> Controller::GetGaitTypeRange() { return GetQrParam().paramRange.gaitType; }

RetState Controller::SetHoldFlag(bool set)
{
    RetState ret;
    return RpcTrySend("qr::SetHoldFlag", set, &ret) == true ? ret : RetState::timeout;
}
bool Controller::GetHoldFlag()
{
    GetCtrlData();
    return ctrlData_.holdFlag;
}

std::string Controller::GetMessage()
{
    auto ret = MsgTryRecv<string>("qr::GetMessage", this);
    if (ret) {
        return ret.value();
    }
    return "";
}

RetState Controller::SetMessage(const std::string &msg)
{
    RetState ret;
    return RpcTrySend("qr::SetMessage", msg, &ret) == true ? ret : RetState::timeout;
}

/**
 * @description: 获取范围，该函数是同一个接口
 * @return {}
 */
ParamRange Controller::GetParamRange()
{
    auto ret = MsgTryRecv<ParamRange>("qr::GetParamRange", this);
    if (ret) {
        paramRange_ = ret.value();
    }
    return paramRange_;
}

void Controller::GetCtrlData()
{
    auto ret = MsgTryRecv<ControllerData>("qr::ControllerData", this);
    if (ret) {
        ctrlData_ = ret.value();
    }
}

/**
 * @description: 中止/暂停当前的阻塞任务
 * 仅对阻塞任务有效
 * 具体是中止还是暂停，是由当前被中止的任务决定
 * @return {}
 */
RetState Controller::PauseCurrentTask()
{
    RetState resp = RetState::timeout;
    RpcTrySend("qr::pauseCurTask", &resp);
    return resp;
}

/**
 * @description: 对外的跳舞API
 * @return {}
 */
RetState Controller::SetDance(int idx)
{
    if (GetQrParam().model.contains(QrModel::middleV3) == true || GetQrParam().model.back() == QrModel::middle) {
        return RetState::noSupport;
    }

    GetCtrlData();
    // 非stand不发送，防止进入stand后运行
    if (ctrlData_.state != RunState::stand) {
        LOG_INFO("now runstate is not stand, dance can't run");
        return RetState::noSupport;
    }

    LOG_INFO("dance start {}", TimerTools::GetNowTickMs());

    QrTask dance;

    switch (idx) {
        case 0: {
        return dance.singleDance0();
    }
        case 1: {
            return dance.singleDance1();  // 注意，角度和频率组合如果不合适，会造成腿之间碰撞和干涉
    }
        case 2: {
        return dance.singleDance2();
    }
        case 3: {
        return dance.singleDance3();
    }
        case 4: {
        return dance.singleDance4();
    }
        case 5: {
        return dance.mixedDance0();
    }
        case 6: {
        return dance.mixedDance1();
    }
    default:
        break;
    }
    return RetState::noSupport;
}
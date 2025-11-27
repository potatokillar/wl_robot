#include "apiQuadruped.hpp"

#include "baseline.hpp"
#include "deviceCustomParam.hpp"
#include "deviceParam.hpp"
#include "qrMotorGazebo.hpp"
#include "qrMotorMit.hpp"
#include "robotMedia.hpp"
#include "robotState.hpp"
#include "spi2can.hpp"

using namespace std;

void ApiQuadruped::Start(QuadBootMode mode)
{
    if (isRun_) {
        return;
    }

    if (mode == QuadBootMode::qr) {
        contr_.Start(bootArgs);
        SetRobotCurState(RobotState::running);
    } else {
        SetRobotCurState(RobotState::jointCtrl);
    }

    isRun_ = mode;
}
void ApiQuadruped::Stop()
{
    if (isRun_) {
        if (isRun_.value() == QuadBootMode::qr) {
            contr_.Stop();
        }
    }
    isRun_.reset();
    SetRobotCurState(RobotState::standby);
}

bool ApiQuadruped::IsRun() { return contr_.IsRun(); }

/**
 * @description: 复位，将所有值都复位成初始值
 * @return {}
 */
void ApiQuadruped::Reset()
{
    linearV_.fill(0);
    angularV_.fill(0);
    pose_.fill(0);
    holdFlag_ = true;
    height_ = 0;
    mode_ = WalkMode::aiNormal;
    gait_ = GaitType::tort;
    nextState_ = RunState::lie;
    contr_.SetLinearVelocityRatio(0, 0, 0);
    contr_.SetAngularVelocityRatio(0, 0, 0);
    contr_.SetPoseRatio(0, 0, 0);
}

/**
 * @description: 设置线速度
 * @param x x方向，m/s
 * @param y y方向，m/s
 * @param z z方向，m/s
 * @return {}
 */
RetState ApiQuadruped::SetLinearVelocity(double x, double y, double z) { return contr_.SetLinearVelocity(x, y, z); }

/**
 * @description: 按比例设置线速度
 * @param x -1到1之间的值
 * @param y
 * @param z
 * @return {}
 */
RetState ApiQuadruped::SetLinearVelocityRatio(double x, double y, double z)
{
    auto err = contr_.SetLinearVelocityRatio(x, y, z);
    if (err == RetState::timeout) {
        // 算法起来之后，出现超时表示算法主线程非预期的阻塞了，是致命错误
        if (errPrint_) {
            LOG_ERROR("SetLinearVelocityRatio failed ERR_TIMEOUT");
        }
    }
    return err;
}
/**
 * @description: 根据上一次的速率修改
 * @param x_diff x差异，可正可负
 * @param y_diff
 * @param z_diff
 * @return {}
 */
RetState ApiQuadruped::ModifyLinearVelocity(double x_diff, double y_diff, double z_diff)
{
    linearV_(0) += x_diff;
    linearV_(1) += y_diff;
    linearV_(2) += z_diff;

    auto err = contr_.SetLinearVelocity(linearV_(0), linearV_(1), linearV_(2));
    if (err == RetState::outRange) {
        LOG_WARN("ModifyLinearVelocity failed");
        linearV_(0) -= x_diff;
        linearV_(1) -= y_diff;
        linearV_(2) -= z_diff;
    }
    return err;
}
/**
 * @description: 获取线速度
 * @param &x
 * @param &y
 * @param &z
 * @return {} 返回为true表示成功获取
 */
std::tuple<double, double, double> ApiQuadruped::GetLinearVelocity()
{
    auto linearV = contr_.GetLinearVelocity();
    return {linearV.x, linearV.y, linearV.z};
}
/**
 * @description: 获取线速度的范围
 * @param &x
 * @param &y
 * @param &z
 * @return {}
 */
std::tuple<ValueRange<double>, ValueRange<double>, ValueRange<double>> ApiQuadruped::GetLinearVelocityRange()
{
    auto range = contr_.GetLinearVelocityRange();
    return {range.x, range.y, range.z};
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * @description: 设置角速度
 * @param x x方向，rad/s
 * @param y y方向，rad/s
 * @param z z方向，rad/s
 * @return {}
 */
RetState ApiQuadruped::SetAngularVelocity(double x, double y, double z)
{
    auto err = contr_.SetAngularVelocity(x, y, z);
    if (err == RetState::outRange) {
        // LOG_WARN("SetAngularVelocity failed");
    }
    return err;
}

/**
 * @description: 设置角速度
 * @param x x方向，rad/s
 * @param y y方向，rad/s
 * @param z z方向，rad/s
 * @return {}
 */
RetState ApiQuadruped::SetAngularVelocityRatio(double x, double y, double z)
{
    auto err = contr_.SetAngularVelocityRatio(x, y, z);
    if (err == RetState::timeout) {
        // 算法起来之后，出现超时表示算法主线程非预期的阻塞了，是致命错误
        if (errPrint_) {
            LOG_ERROR("SetAngularVelocityRatio failed ERR_TIMEOUT");
        }
    }
    return err;
}

/**
 * @description: 修改线速度
 * @param x_diff
 * @param y_diff
 * @param z_diff
 * @return {}
 */
RetState ApiQuadruped::ModifyAngularVelocity(double x_diff, double y_diff, double z_diff)
{
    angularV_(0) += x_diff;
    angularV_(1) += y_diff;
    angularV_(2) += z_diff;

    auto err = contr_.SetAngularVelocity(angularV_(0), angularV_(1), angularV_(2));
    if (err == RetState::outRange) {
        LOG_WARN("ModifyAngularVelocity failed");
        angularV_(0) -= x_diff;
        angularV_(1) -= y_diff;
        angularV_(2) -= z_diff;
    }
    return err;
}

/**
 * @description: 获取角速度
 * @param &x
 * @param &y
 * @param &z
 * @return {} 返回为true表示成功获取
 */
std::tuple<double, double, double> ApiQuadruped::GetAngularVelocity()
{
    auto angularV = contr_.GetAngularVelocity();
    return {angularV.x, angularV.y, angularV.z};
}

std::tuple<ValueRange<double>, ValueRange<double>, ValueRange<double>> ApiQuadruped::GetAngularVelocityRange()
{
    auto range = contr_.GetAngularVelocityRange();
    return {range.x, range.y, range.z};
}

///////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * @description: 设置位置
 * @param roll
 * @param pitch
 * @param yaw
 * @return {}
 */
RetState ApiQuadruped::SetPose(double roll, double pitch, double yaw)
{
    auto err = contr_.SetPose(roll, pitch, yaw);
    if (err == RetState::outRange) {
        // LOG_WARN("SetPose failed: {}, {}, {}", roll, pitch, yaw);
    }
    return err;
}
RetState ApiQuadruped::SetPoseRatio(double roll, double pitch, double yaw)
{
    auto err = contr_.SetPoseRatio(roll, pitch, yaw);
    if (err == RetState::timeout) {
        // 算法起来之后，出现超时表示算法主线程非预期的阻塞了，是致命错误
        if (errPrint_) {
            LOG_ERROR("SetPoseRatio failed ERR_TIMEOUT");
        }
    }
    return err;
}

/**
 * @description: 设置姿态
 * @param roll_diff
 * @param pitch_diff
 * @param yaw_diff
 * @return {}
 */
RetState ApiQuadruped::ModifyPose(double roll_diff, double pitch_diff, double yaw_diff)
{
    pose_(0) += roll_diff;
    pose_(1) += pitch_diff;
    pose_(2) += yaw_diff;

    auto err = contr_.SetPose(pose_(0), pose_(1), pose_(2));
    if (err == RetState::outRange) {
        LOG_WARN("ModifyPose failed");
        pose_(0) -= roll_diff;
        pose_(1) -= pitch_diff;
        pose_(2) -= yaw_diff;
    }
    return err;
}

/**
 * @description: 获取姿态
 * @param &roll
 * @param &pitch
 * @param &yaw
 * @return {}
 */
std::tuple<double, double, double> ApiQuadruped::GetPose()
{
    auto pose = contr_.GetPose();
    return {pose.roll, pose.pitch, pose.yaw};
}
std::tuple<ValueRange<double>, ValueRange<double>, ValueRange<double>> ApiQuadruped::GetPoseRange()
{
    auto range = contr_.GetPoseRange();
    return {range.roll, range.pitch, range.yaw};
}

//////////////////////////////////////////////////////////////////////////////

/**
 * @description: 设置高度偏差
 * @param heightOffset
 * @return {}
 */
RetState ApiQuadruped::SetHeight(double heightOffset)
{
    auto err = contr_.SetHeight(heightOffset);
    if (err == RetState::outRange) {
        //  LOG_WARN("SetHeight failed");
    }
    return err;
}
RetState ApiQuadruped::SetHeightRatio(double heightRatio)
{
    auto err = contr_.SetHeightRatio(heightRatio);
    if (err == RetState::timeout) {
        // 算法起来之后，出现超时表示算法主线程非预期的阻塞了，是致命错误
        if (errPrint_) {
            LOG_ERROR("SetHeightRatio failed ERR_TIMEOUT");
        }
    }
    return err;
}

/**
 * @description: 按比例修改高度
 * @param height_diff 值限定，小于0.02不行
 * @return {}
 */
RetState ApiQuadruped::ModifyHeightRatio(double height_diff)
{
    auto err = contr_.SetHeightRatio(height_diff);
    if (err == RetState::timeout) {
        // 算法起来之后，出现超时表示算法主线程非预期的阻塞了，是致命错误
        //  LOG_ERROR("ModifyHeightRatio failed ERR_TIMEOUT");
    }
    return err;
}

/**
 * @description: 修改高度偏差值
 * @param height_diff
 * @return {}
 */
RetState ApiQuadruped::ModifyHeight(double height_diff)
{
    const auto& range = contr_.GetHeightRange();
    auto set = height_ + height_diff;

    if ((set > range.max) || (set < range.min)) {
        return RetState::outRange;
    }

    auto err = contr_.SetHeight(set);
    if (err == RetState::ok) {
        height_ = set;
    }

    return err;
}
/**
 * @description: 获取高度偏差
 * @param &heightOffset
 * @return {}
 */
double ApiQuadruped::GetHeight() { return contr_.GetHeight(); }
ValueRange<double> ApiQuadruped::GetHeightRange() { return contr_.GetHeightRange(); }

///////////////////////////////////////////////////////////////////

/**
 * @description: 设置运行状态机，todo该逻辑和ctrlApi的SetRunStateExt有混乱的可能
 * @param sta
 * @return {}
 */
RetState ApiQuadruped::SetRunState(RunState sta)
{
    // 若请求stand状态，但算法未启动，则先启动再运行
    if ((IsRun() == false) && (sta == RunState::stand)) {
        Start(QuadBootMode::qr);
        TimerTools::SleepForS(1);
    }

    auto err = contr_.SetRunState(sta);
    if (err == RetState::ok) {
        // 一切顺利，则保存
        nextState_ = sta;
    }

    return err;
}
/**
 * @description: 获取运行状态
 * @param &sta
 * @return {}
 */
RunState ApiQuadruped::GetRunState() { return contr_.GetRunState(); }

std::vector<RunState> ApiQuadruped::GetRunStateRange()
{
    return contr_.GetRunStateRange();
    ;
}

///////////////////////////////////////////////////////////////////////////////////////

/**
 * @description: 设置walkmode
 * @param set
 * @return {}
 */
RetState ApiQuadruped::SetWalkMode(WalkMode mode)
{
    auto err = contr_.SetWalkMode(mode);
    if (err == RetState::noSupport) {
        LOG_WARN("SetWalkMode failed");
        return err;
    }
    mode_ = mode;
    return err;
}

/**
 * @description: 修改运行模式
 * @return {}
 */
RetState ApiQuadruped::ModifyWalkMode()
{
    /*
    if (mode_ == WalkMode::normal) {
        mode_ = WalkMode::climb;
    } else if (mode_ == WalkMode::climb) {
        mode_ = WalkMode::light;
    } else if (mode_ == WalkMode::light) {
        mode_ = WalkMode::ai;
    } else {
        mode_ = WalkMode::normal;
    }
    */

    if (mode_ == WalkMode::aiNormal) {
        mode_ = WalkMode::aiClimb;
    } else if (mode_ == WalkMode::aiClimb) {
        mode_ = WalkMode::aiNormal;
    }

    auto err = contr_.SetWalkMode(mode_);
    if (err == RetState::noSupport) {
        LOG_WARN("ModifyWalkMode failed");
    }
    return err;
}

RetState ApiQuadruped::ModifyWalkModeReverse()
{
    // 只保留普通模式和AI模式
    if (mode_ == WalkMode::aiNormal) {
        mode_ = WalkMode::aiClimb;
    } else if (mode_ == WalkMode::aiClimb) {
        mode_ = WalkMode::aiNormal;
    }

    auto err = contr_.SetWalkMode(mode_);
    if (err == RetState::noSupport) {
        LOG_WARN("ModifyWalkMode failed");
    }
    return err;
}

/**
 * @description: 设置步行模式
 * @param &mode
 * @return {}
 */
WalkMode ApiQuadruped::GetWalkMode() { return contr_.GetWalkMode(); }
std::vector<WalkMode> ApiQuadruped::GetWalkModeRange() { return contr_.GetWalkModeRange(); }

///////////////////////////////////////////////////////////////////////////////////////////

/**
 * @description: 设置步态
 * @param gait
 * @return {}
 */
RetState ApiQuadruped::SetGaitType(GaitType gait)
{
    auto err = contr_.SetGaitType(gait);
    if (err == RetState::noSupport) {
        LOG_WARN("SetGaitType failed");
        return err;
    }

    gait_ = gait;
    return err;
}
/**
 * @description: 修改步态,todo，目前只有一个步态，因此这里未实现
 * @return {}
 */
RetState ApiQuadruped::ModifyGaitType()
{
    auto err = contr_.SetGaitType(gait_);
    if (err == RetState::noSupport) {
        LOG_WARN("ModifyGaitType failed");
        return err;
    }
    return err;
}
/**
 * @description: 获取步态
 * @param &gait
 * @return {}
 */
GaitType ApiQuadruped::GetGaitType() { return contr_.GetGaitType(); }
std::vector<GaitType> ApiQuadruped::GetGaitTypeRange() { return contr_.GetGaitTypeRange(); }

/////////////////////////////////////////////////////////////////////////////////////

double ApiQuadruped::GetLoadMass() { return contr_.GetLoadMass(); }
RetState ApiQuadruped::SetLoadMass(double set)
{
#if ARCH == x86
    LOG_TRACE("input param, set = {}", set);
#endif
    set = std::round(set * 100.0) / 100.0;  // 取整
    RetState ret = contr_.SetLoadMass(set);

    double set_100 = std::round(set * 100);  // 乘100，消除小数位数
    int intPart = static_cast<int>(set_100);

#if ARCH == x86
    LOG_TRACE("std::round, set = {}", set);
    LOG_TRACE("std::round and plus 100, set = {}", set_100);
    LOG_TRACE("intPart = {}", intPart);
#endif

    if (ret == RetState::ok) {  // 仅在设置成功，不出现超限问题的情况下，才进行语音播报
        MediaTaskPackage media_pkg;
        media_pkg.Add("setLoadMass");
        std::vector<int> digits;  // 存储各位数的容器
        if (intPart == 0) {       // 等于0的情况
            digits.push_back(0);
            media_pkg.Add("0");
        } else {
            // 从个位开始，逐个数字存放到容器中，再逆序容器
            while (intPart != 0) {
                digits.push_back(intPart % 10);
                intPart /= 10;
            }
            std::reverse(digits.begin(), digits.end());
            // 补齐0和小数点
            while (digits.size() < 3) {
                digits.insert(digits.begin(), 0);
            }
            // 删除小数位中多余的0，以及小数点
            for (size_t i = 0; i < digits.size(); i++) {
                if (i == digits.size() - 2) {
                    if (digits.at(i) == 0 && digits.at(i + 1) == 0) {
                        continue;
                    }
                    media_pkg.Add("point");
                } else if (i == digits.size() - 1) {
                    if (digits.at(i) == 0) {
                        continue;
                    }
                }
                media_pkg.Add(to_string(digits.at(i)));
            }
        }
        media_pkg.Add("kg");
        AddMediaPackage(media_pkg);
#if ARCH == x86  // 打印digits容器里的内容
        std::stringstream ss;
        for (auto i : digits) {
            ss << to_string(i) << " ";
        }
        LOG_TRACE("digits = {}", ss.str());
#endif
    }
    return ret;
}
RetState ApiQuadruped::ModifyLoadMass(double set_diff)
{
    loadMass_ += set_diff;
    auto err = SetLoadMass(loadMass_);
    if (err != RetState::ok) {
        loadMass_ -= set_diff;
        return err;
    }
    return err;
}
ValueRange<double> ApiQuadruped::GetLoadMassRange() { return contr_.GetLoadMassRange(); }

//////////////////////////////////////////////////////////////////////////////////////////

RetState ApiQuadruped::SetHoldFlag(bool set)
{
    auto err = contr_.SetHoldFlag(set);
    if (err != RetState::ok) {
        LOG_WARN("SetHoldFlag failed");
        return err;
    }

    holdFlag_ = set;
    return err;
}
RetState ApiQuadruped::ModifyHoldFlag()
{
    auto err = contr_.SetHoldFlag(!holdFlag_);
    if (err != RetState::ok) {
        LOG_WARN("SetHoldFlag failed");
        return err;
    }

    holdFlag_ = !holdFlag_;
    return err;
}

bool ApiQuadruped::GetHoldFlag() { return contr_.GetHoldFlag(); }

RetState ApiQuadruped::SetDance(int idx) { return contr_.SetDance(idx); }

void ApiQuadruped::SetEmergStop(bool set)
{
    if (set) {
        contr_.Stop();
        SetRobotCurState(RobotState::emgstop);
    } else {
        if (GetRobotCurState() == RobotState::emgstop) {
            SetRobotCurState(RobotState::standby);
        }
    }
}

void ApiQuadruped::SendMotorCmd(const msg::qr::motor_cmd& cmd) { MsgTrySend("qr::motor_cmd", cmd); }
msg::qr::motor_ret ApiQuadruped::RecvMotorRet()
{
    auto cmd = MsgTryRecv<msg::qr::motor_ret>("qr::motor_ret", this);
    if (cmd) {
        motorDataPrev_ = cmd.value();
    }
    return motorDataPrev_;
}

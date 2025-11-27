
#pragma once
#include "baseline.hpp"
#include "controller.hpp"
#include "qrMotor.hpp"

// 启动模式，是算法模式还是关节控制模式
enum class QuadBootMode
{
    qr,
    joint,
};
class ApiQuadruped
{
public:
    void Start(QuadBootMode mode);
    void Stop();
    void Reset();
    bool IsRun();

    // 线速度相关
    RetState SetLinearVelocity(double x, double y, double z);
    RetState SetLinearVelocityRatio(double x, double y, double z);
    RetState ModifyLinearVelocity(double x_diff, double y_diff, double z_diff);
    std::tuple<double, double, double> GetLinearVelocity();
    std::tuple<ValueRange<double>, ValueRange<double>, ValueRange<double>> GetLinearVelocityRange();

    // 角速度相关
    RetState SetAngularVelocity(double x, double y, double z);
    RetState SetAngularVelocityRatio(double x, double y, double z);
    RetState ModifyAngularVelocity(double x_diff, double y_diff, double z_diff);
    std::tuple<double, double, double> GetAngularVelocity();
    std::tuple<ValueRange<double>, ValueRange<double>, ValueRange<double>> GetAngularVelocityRange();

    // 姿态相关
    RetState SetPose(double roll, double pitch, double yaw);
    RetState SetPoseRatio(double roll, double pitch, double yaw);
    RetState ModifyPose(double roll_diff, double pitch_diff, double yaw_diff);
    std::tuple<double, double, double> GetPose();
    std::tuple<ValueRange<double>, ValueRange<double>, ValueRange<double>> GetPoseRange();

    // 高度相关
    RetState SetHeight(double height);
    RetState SetHeightRatio(double height);
    RetState ModifyHeight(double height_diff);
    RetState ModifyHeightRatio(double height_diff);
    double GetHeight();
    ValueRange<double> GetHeightRange();

    RetState SetRunState(RunState sta);
    RunState GetRunState();
    std::vector<RunState> GetRunStateRange();

    RetState SetWalkMode(WalkMode mode);
    RetState ModifyWalkMode();
    RetState ModifyWalkModeReverse();
    WalkMode GetWalkMode();
    std::vector<WalkMode> GetWalkModeRange();

    RetState SetGaitType(GaitType gait);
    RetState ModifyGaitType();
    GaitType GetGaitType();
    std::vector<GaitType> GetGaitTypeRange();

    RetState SetHoldFlag(bool set);
    RetState ModifyHoldFlag();
    bool GetHoldFlag();

    RetState SetLoadMass(double set);
    RetState ModifyLoadMass(double set_diff);
    double GetLoadMass();
    ValueRange<double> GetLoadMassRange();

    RetState SetDance(int idx);

    void SetEmergStop(bool set);

    void SendMotorCmd(const msg::qr::motor_cmd& cmd);  // 0524实现该功能
    msg::qr::motor_ret RecvMotorRet();

private:
    Controller contr_;
    bool errPrint_{false};
    Vec3<double> linearV_{0, 0, 0};
    Vec3<double> angularV_{0, 0, 0};
    Vec3<double> pose_{0, 0, 0};
    bool holdFlag_{false};
    double height_{0};
    double loadMass_{0};
    WalkMode mode_{WalkMode::aiNormal};
    GaitType gait_{GaitType::tort};
    RunState nextState_{RunState::lie};
    msg::qr::motor_ret motorDataPrev_;

    std::optional<QuadBootMode> isRun_;
};
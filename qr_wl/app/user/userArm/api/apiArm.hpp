
#pragma once
#include "apiArmCtrl.hpp"
#include "driver.hpp"
#include "gripper.hpp"
class ApiArm
{
public:
    ~ApiArm();
    void Start();
    void Stop();
    bool IsRun();

    Result<u32> SetMoveJTaskNoBlock(const std::vector<std::vector<double>>& jointEnd, MoveMode movemode);
    Result<u32> SetMoveJTaskNoBlock(const std::vector<std::vector<double>>& jointEnd, MoveMode movemode, double spd);
    Result<u32> SetMoveJTaskNoBlock(const std::vector<std::vector<double>>& jointEnd, MoveMode movemode, double spd, double acc);

    Result<u32> SetMoveLTaskNoBlock(const std::vector<Pose>& posEnd, MoveMode movemode);
    Result<u32> SetMoveLTaskNoBlock(const std::vector<Pose>& posEnd, MoveMode movemode, double spd);
    Result<u32> SetMoveLTaskNoBlock(const std::vector<Pose>& posEnd, MoveMode movemode, double spd, double acc);

    RetState WaitMoveTaskComplete(u32 timeout);

    RetState SetMoveJTaskBlock(const std::vector<double>& jointEnd, MoveMode movemode, double spd, double acc);
    RetState SetMoveLTaskBlock(const Pose& posEnd, MoveMode movemode, double spd, double acc);

    RetState SetCtrlState(arm::ArmCtrl set);

    std::vector<std::string> GetToolNameList();
    RetState SetToolName(const std::string& name);
    std::string GetToolName();
    RetState SetToolValue(const std::string& toolName, const Pose& pose);
    Result<Pose> GetToolValue(std::string toolName);

    std::vector<double> GetNowAngle();
    std::vector<double> GetNowVelocity();
    Pose GetNowCart();
    std::vector<double> GetNowCartSpeed();

    arm::ArmState GetArmState();

    Pose GetFkine6(std::vector<double> robotjoint);
    std::pair<bool, Vec6<double>> GetIkine6(Pose FK, std::vector<double> NowAngle);

    std::pair<Pose, double> GetFkine7(std::vector<double> robotjoint);
    std::pair<bool, Vec7<double>> GetIkine7(Pose FK, double phi, std::vector<double> NowAngle);

    Result<Mat6<double>> GetJacobianTool6(std::vector<double> robotjoint);
    Result<Mat6<double>> GetJacobianBase6(std::vector<double> robotjoint);

    RetState SetIoOutState(const std::string& name, std::bitset<32> set);
    std::bitset<32> GetIoInState(const std::string& name);
    std::bitset<32> GetIoOutState(const std::string& name);

    Result<std::vector<ValueRange<double>>> GetPositionLimit();
    RetState SetPositionLimit(std::vector<ValueRange<double>> data);
    // RetState SetUserTool(std::pair<std::string, Pose> data);
    RetState SetCollisionLevel(u8 level);
    Result<u8> GetCollisionLevel();

    RetState SetReset(bool Reset);
    Result<bool> GetReset();

    RetState SetRapidrate(double rate);
    Result<double> GetRapidrate();

    RetState SetDragMode(bool mode);
    Result<bool> GetDragMode();

    RetState SetJointSpeedLimit(std::vector<double> Vmax);
    Result<std::vector<double>> GetJointSpeedLimit();

    RetState SetJointAccLimit(std::vector<double> Amax);
    Result<std::vector<double>> GetJointAccLimit();

    std::vector<double> GetAxisCurrent();
    std::vector<double> GetAxisVoltage();
    std::vector<double> GetAxisTemperature();
    std::vector<double> GetAxisTorque();
    RetState SetGripper(bool state, double speed);

    Result<Vec6<double>> GetJacobian();

private:
    arm::ApiArmCtrl api_;
    std::shared_ptr<ArmMotor> motor_;
    ControlGripperAPI gripperAPI_;
};
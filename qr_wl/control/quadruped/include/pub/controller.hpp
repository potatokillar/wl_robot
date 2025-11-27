
#pragma once

#include <condition_variable>

#include "baseline.hpp"
#include "contrType.hpp"

class Controller
{
public:
    void Start(const BootArgs &args);
    void Stop();
    bool IsRun();

    Controller();
    ~Controller() = default;

    // 控制算法的接口，现在单独区分了
    // RetState SetCtrlData(const ControllerCmd &cmd);
    bool GetCtrlData(ControllerData *data) const;  // 获取算法的数据

    /**
     * @description: 向算法填写IMU数据
     * @param &data
     * @return {}
     */
    void SetImuData(const msg::imu_data &data);
    /**
     * @description: 向算法填写电机返回数据
     * @param &data
     * @return {}
     */
    void SetMotorData(const msg::qr::motor_ret &data);
    /**
     * @description: 从算法获取本次的电机控制指令
     * @param *cmd
     * @return {}
     */
    bool ret2flagX(msg::qr::motor_cmd *cmd);

    // 线速度相关接口
    RetState SetLinearVelocity(double x, double y, double z);
    VelocityType<double> GetLinearVelocity();
    VelocityType<ValueRange<double>> GetLinearVelocityRange();
    RetState SetLinearVelocityRatio(double x, double y, double z);

    // 角速度相关
    RetState SetAngularVelocity(double x, double y, double z);
    VelocityType<double> GetAngularVelocity();
    VelocityType<ValueRange<double>> GetAngularVelocityRange();
    RetState SetAngularVelocityRatio(double x, double y, double z);

    // 姿态相关
    RetState SetPose(double roll, double pitch, double yaw);
    RetState SetPoseRatio(double roll, double pitch, double yaw);
    PoseType<double> GetPose();
    PoseType<ValueRange<double>> GetPoseRange();

    // 高度相关
    RetState SetHeight(double height);
    RetState SetHeightRatio(double height);
    double GetHeight();
    ValueRange<double> GetHeightRange();
    double GetHeightRatio();

    // 运行状态机相关
    RetState SetRunState(AlgoState sta);
    AlgoState GetRunState();
    std::vector<AlgoState> GetRunStateRange();

    // 步行模式相关
    RetState SetWalkMode(WalkMode mode);
    WalkMode GetWalkMode();
    std::vector<WalkMode> GetWalkModeRange();

    // 步态相关
    RetState SetGaitType(GaitType gait);
    GaitType GetGaitType();
    std::vector<GaitType> GetGaitTypeRange();

    // holdflag相关
    RetState SetHoldFlag(bool set);
    bool GetHoldFlag();

    // 设置负载
    RetState SetLoadMass(double mass);
    double GetLoadMass();
    ValueRange<double> GetLoadMassRange();

    RetState SetMessage(const std::string &msg);  // 往算法传递数据
    std::string GetMessage();                     // 从算法获取内部数据，注，这并不是SetMessage的值

    RetState PauseCurrentTask();

    RetState SetDance(int idx);

private:
    ControllerData ctrlData_;
    bool isRun_{false};

    double Ratio2Value(double v, ValueRange<double> range);
    double Value2Ratio(double r, ValueRange<double> range);
    ParamRange paramRange_;
    ParamRange GetParamRange();
    void GetCtrlData();
};
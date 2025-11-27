
#pragma once

#include "ArmType.hpp"
#include "baseline.hpp"

namespace arm
{
// 面向用户侧的信息，存放频繁变化的数据
struct ArmUserInfo
{
    ArmState state{ArmState::noRun};
    std::map<std::string, std::bitset<32>> ioInSta;
    std::map<std::string, std::bitset<32>> ioOutSta;
    std::vector<double> angle;
    std::vector<double> velocity;
    Pose cart;
    double Phi;  // 七轴需要使用的臂角
    struct
    {
        std::vector<double> current;
        std::vector<double> voltage;
        std::vector<double> temperature;
        std::vector<double> torque;
    } motor;
};

// struct ArmUserInfo;  // 前置声明，该变量不对外开放，todo编译错误，怀疑是依赖问题
// struct ArmUserParam;

class ApiArmCtrl
{
public:
    ApiArmCtrl();
    /**
     * @description: 启动机械臂算法，启动是否成功通过isRun判断
     * @param args
     * @param motor 电机实例
     * @return {}
     */
    void Start(const BootArgs& args);
    void Stop();

    /**
     * @description: 获取对应的回调函数
     * @return {}
     */
    Result<std::function<std::optional<msg::arm_cmd>(msg::arm_data)>> GetTxRxCallback();
    Result<std::function<void(msg::arm_motor_info)>> GetInfoCallback();

    /**
     * @description: 判断机械臂算法是否启动成功
     * @return {}
     */
    bool IsRun();

    /**
     * @brief 机械臂关节空间规划接口，仅设置不启动
     *
     * @param jointEnd 期望关节角度
     * @param moveMode 运动模式：ABS/INCR
     * @param robSpeed 运行速率 ：0-1
     * @param acc 加速度 ：0-1
     * @return 成功返回执行完成的最长时间，失败返回失败原因
     */
    Result<u32> SetMoveJTask(const std::vector<std::vector<double>>& jointEnd, MoveMode moveMode);
    Result<u32> SetMoveJTask(const std::vector<std::vector<double>>& jointEnd, MoveMode moveMode, double robSpeed);
    Result<u32> SetMoveJTask(const std::vector<std::vector<double>>& jointEnd, MoveMode moveMode, double robSpeed, double acc);

    /**
     * @brief 机械臂笛卡尔空间规划接口，仅设置不启动
     *
     * @param posEnd 期望空间位置
     * @param moveMode 运动模式：ABS/INCR
     * @param robSpeed 运行速率 ：0-1
     * @return 成功返回执行完成的最长时间，失败返回失败原因
     */
    Result<u32> SetMoveLTask(const std::vector<Pose>& posEnd, MoveMode moveMode);
    Result<u32> SetMoveLTask(const std::vector<Pose>& posEnd, MoveMode moveMode, double robSpeed);
    Result<u32> SetMoveLTask(const std::vector<Pose>& posEnd, MoveMode moveMode, double robSpeed, double acc);

    /**
     * @description: 等待movetask运行完毕，即启动SetMoveJTask或SetMoveLTask的运行，超时/中断/完成后返回
     * @param timeout 等待超时时间，单位ms
     * @return {}
     */
    RetState WaitMoveTaskComplete(u32 timeout);

    /**
     * @description: 设置机械臂控制状态
     * @param set
     * @return {}
     */
    RetState SetCtrlState(ArmCtrl set);

    /**
     * @description: 获取机械臂运行状态
     * @return {}
     */
    ArmState GetRunState();

    /**
     * @description: 获取工具坐标系的名字列表，若算法未start，则为空
     * @return {}
     */
    std::vector<std::string> GetToolNameList();

    /**
     * @description: 设置默认工具坐标系名字
     * @return {}
     */
    RetState SetToolName(const std::string& toolName);

    /**
     * @description: 获取当前使用的默认工具坐标系，若算法未启动，则为空
     * @return {}
     */
    std::string GetToolName();

    /**
     * @description: 根据工具坐标系名称查询对应的坐标系内容并完成上报
     * @param toolName 需要获取参数的工具坐标系名称
     * @return {} 对应的工具坐标系的内容，4*4矩阵
     */
    Result<Pose> GetToolValue(const std::string& toolName);

    /**
     * @description: 设置工具坐标系具体值
     * @return {}
     */
    RetState SetToolValue(const std::string& toolName, const Pose& pose);

    /**
     * @brief 获取机械臂当前的实际关节角
     *
     * @return Result<Vec6<double>>
     */

    std::vector<double> GetNowAngle();
    /**
     * @brief 获取机械臂当前的实际关节角速度
     *
     * @return vector<double>
     */
    std::vector<double> GetNowVelocity();

    /**
     * @description: 获取机械臂当前的笛卡尔坐标系
     * @return {}
     */
    Pose GetNowCart();
    std::vector<double> GetNowCartSpeed();
    /**
     * @brief 正运动学函数
     *
     * @param robotjoint 机器人关节角度
     * @param Tool 用户指定的工具坐标系
     * @return Mat4<double>
     */
    Pose GetFkine6(std::vector<double> robotjoint);
    std::pair<Pose, double> GetFkine7(std::vector<double> robotjoint);
    /**
     * @brief 逆运动学函数
     *
     * @param FK 末端笛卡尔空间位置 齐次矩阵形式
     * @return Vec6<double> 关节角
     */
    std::pair<bool, Vec6<double>> GetIkine6(Pose FK, std::vector<double> NowAngle);
    std::pair<bool, Vec7<double>> GetIkine7(Pose FK, double phi, std::vector<double> NowAngle);
    /**
     * @description: IO状态相关
     * @param set
     * @return {}
     */
    RetState SetIoOutState(const std::string& name, std::bitset<32> set);
    std::bitset<32> GetIoInState(const std::string& name);
    std::bitset<32> GetIoOutState(const std::string& name);

    /**
     * @description: 获取关节限位
     * @return {}
     */
    Result<ValueRange<std::vector<double>>> GetPositionLimit();
    /**
     * @description: 设置关节限位
     * @return {}
     */
    RetState SetPositionLimit(std::vector<ValueRange<double>> data);
    /**
     * @description: 设置超限复位标志
     * @return {}
     */

    // RetState SetUserTool(std::pair<std::string, Pose> data);

    RetState SetReset(bool Reset);
    Result<bool> GetReset();
    RetState SetRapidrate(double rate);
    Result<double> GetRapidrate();

    RetState SetJointSpeedLimit(std::vector<double> Vmax);
    Result<std::vector<double>> GetJointSpeedLimit();

    RetState SetJointAccLimit(std::vector<double> Amax);
    Result<std::vector<double>> GetJointAccLimit();

    std::vector<double> GetAxisCurrent();
    std::vector<double> GetAxisVoltage();
    std::vector<double> GetAxisTemperature();
    std::vector<double> GetAxisTorque();

    RetState SetCollisionLevel(u8 level);
    Result<u8> GetCollisionLevel();

    RetState SetDragMode(bool mode);
    Result<bool> GetDragMode();
    Result<ArmUserParam> GetUserParam();
    Result<Mat6<double>> GetJacobianTool6(std::vector<double> robotjoint);
    Result<Mat6<double>> GetJacobianBase6(std::vector<double> robotjoint);

private:
    bool isRun_{false};
    void Reset();
    std::unique_ptr<ArmUserInfo> info_;

    /**
     * @description: 更新信息，需要主动调用，刷新
     * @return {}
     */
    void UpdateInfo();
};
}  // namespace arm

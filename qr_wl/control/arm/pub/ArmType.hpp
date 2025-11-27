
#pragma once

#include <iostream>

#include "baseline.hpp"

/**
 * @brief 笛卡尔空间位置
 * @param x X 轴坐标
 * @param y Y 轴坐标
 * @param z Z 轴坐标
 */
struct CartesianTran
{
    double x, y, z;
};
/**
 * @brief 欧拉角
 * @param rx 绕固定轴 X 旋转角度
 * @param ry 绕固定轴 Y 旋转角度
 * @param rz 绕固定轴 Z 旋转角度
 */
struct Rpy
{
    double rx, ry, rz;
};
/**
 * @brief 笛卡尔空间位姿类型
 * 例：RPY = 3.14,-1.57,1.57;旋转顺序为先将工具座标系绕着其自身的Z轴旋转1.57，再绕着旋转后的自身Y轴旋转-1.57，最后绕再次旋转后的自身X轴旋转3.14
 * @param tran_ 笛卡尔空间位置
 * @param rpy_ 笛卡尔空间姿态
 */
struct Pose
{
    CartesianTran tran_;
    Rpy rpy_;

    Pose(double x, double y, double z, double rx, double ry, double rz)
    {
        tran_.x = x;
        tran_.y = y;
        tran_.z = z;
        rpy_.rx = rx;
        rpy_.ry = ry;
        rpy_.rz = rz;
    }
    Pose()
    {
        tran_.x = 0.0;
        tran_.y = 0.0;
        tran_.z = 0.0;
        rpy_.rx = 0.0;
        rpy_.ry = 0.0;
        rpy_.rz = 0.0;
    }
    void setZero()
    {
        tran_.x = 0.0;
        tran_.y = 0.0;
        tran_.z = 0.0;
        rpy_.rx = 0.0;
        rpy_.ry = 0.0;
        rpy_.rz = 0.0;
    }
    void PrintPose() const { std::cout << "Pose: " << tran_.x << "," << tran_.y << "," << tran_.z << "," << rpy_.rx << "," << rpy_.ry << "," << rpy_.rz << std::endl; }
};
/**
 * @brief 四元数
 * @param w
 * @param x
 * @param y
 * @param z
 */
struct Quaternion
{
    double w{1};
    double x{0};
    double y{0};
    double z{0};
};

/**
 * @brief 运动模式
 * @param abs 绝对位置
 * @param incr 增量位置
 */
enum class MoveMode
{
    abs,
    incr,
};
/**
 * @brief 速度模式
 * @param abs 绝对速度
 * @param prop 速度比例
 */
enum class SpeedMode
{
    abs,
    prop,
};
/**
 * @brief
 * @param movej 关节空间运动
 * @param movel 笛卡尔空间运动
 */
enum class MoveFlag
{
    movej,  // 关节空间运动
    movel,  // 笛卡尔空间运动
};

/**
 * @brief
 * @param Targetjoint_ 关节空间位置
 * @param Targetpose_ 笛卡尔空间位置
 * @param flag_ 区分关节空间与笛卡尔空间
 * @param speed_ 速度
 * @param movemode_ 运动模式
 * @param Ttool_ 工具坐标系
 * @param Tworld_ 世界坐标系
 */
struct ArmMovePoint
{
    std::any Targetjoint;
    Pose Targetpose;
    MoveFlag flag;
    double speed;
    double acceleration;
    MoveMode movemode;
    SpeedMode speedmode;
    double rapidrate;
};

// 面向用户侧的参数，存放较少变化的数据
struct ArmUserParam
{
    std::vector<double> Smax;
    std::vector<double> Smin;
    std::vector<double> Vmax;
    std::vector<double> Amax;
    std::map<std::string, Pose> userTool;
    double rapidrate;
    bool reset;
};

struct ArmUserTool
{
    std::string toolName;
    Pose toolPose;
};

struct PlanResults
{
    double Y;
    double A;
    double J;
};

namespace arm
{
// 机械臂控制指令
enum class ArmCtrl
{
    enable,   // 使能
    disable,  // 失能
    start,    // 启动
    pause,    // 暂停
    resume,   // 恢复
    stop,     // 停止
    emgStop,  // 急停
};

// 机械臂状态
enum class ArmState
{
    noRun,      // 未运行算法
    init,       // 初始化中，不可操作
    noReady,    // 未就绪，需要下发enable让其就绪
    ready,      // 准备就绪，可以下发新任务
    running,    // 正在执行任务中
    pause,      // 任务被暂停，可start重新启动上次任务，也可stop完全放弃该任务
    complete,   // 任务正常完成，一段时间后自动切成ready
    interrupt,  // 任务被中断，一段时间后自动切成ready
    error,
};

enum class ArmErrState
{
    outRange,  // 关节运动超限
};

}  // namespace arm

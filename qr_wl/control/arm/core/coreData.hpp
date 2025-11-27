
#pragma once
#include "apiArmCtrl.hpp"
#include "baseline.hpp"

// 算法模块用命名空间包含，阻止可能的命名污染
namespace arm
{
struct CoreData
{
    Vec6<double> nowAngle;  // 当前角度
    Vec6<double> nowVelocity;
    Vec7<double> nowAngle_seven;
    Vec7<double> nowVelocity_seven;
    std::string toolName{"tool0"};  // 默认工具系

    // 用户状态，大部分和电机状态基本对应，小部分由算法自行更改
    arm::ArmState armSta{arm::ArmState::noRun};

    msg::arm_motor_info motorInfo;  // 当前电机相关信息
};
}  // namespace arm
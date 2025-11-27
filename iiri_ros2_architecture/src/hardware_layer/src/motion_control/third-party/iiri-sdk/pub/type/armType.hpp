/*
 * @Author: 唐文浩
 * @Date: 2025-02-28
 * @LastEditors: 唐文浩
 * @LastEditTime: 2025-02-28
 * @Description: 机械臂类型
 *
 * Copyright (c) 2022-2025 by 西湖大学智能产业研究院, All Rights Reserved.
 */
#pragma once
#include <array>

#include "commType.hpp"

namespace iiri::arm
{
    static constexpr uint8_t ARM_MOTOR_SIZE = 7; // 7轴，兼容6轴

    using MotorCmd = std::array<iiri::MitMotorCmd, ARM_MOTOR_SIZE>;
    using MotorRet = std::array<iiri::MitMotorRet, ARM_MOTOR_SIZE>;

    struct ArmTeach
    {
        double x;
        double y;
        double z;
        double Rx;
        double Ry;
        double Rz;
    };

} // namespace iiri::arm
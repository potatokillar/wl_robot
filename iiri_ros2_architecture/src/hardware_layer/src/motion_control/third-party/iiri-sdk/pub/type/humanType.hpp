/*
 * @Author: 唐文浩
 * @Date: 2025-02-28
 * @LastEditors: 唐文浩
 * @LastEditTime: 2025-02-28
 * @Description:
 *
 * Copyright (c) 2022-2025 by 西湖大学智能产业研究院, All Rights Reserved.
 */
#pragma once
#include <array>

#include "commType.hpp"

namespace iiri::human
{
    constexpr int LEG_MOTOR_NUM = 5; // 单腿电机数目
    constexpr int ARM_MOTOR_NUM = 6; // 手臂电机数目

    // 人型所有电机的集合，有namespace包含的不需要重复增加human前缀
    template <typename T>
    struct MotorContainer
    {
        std::array<std::array<T, human::LEG_MOTOR_NUM>, 2> leg; // 左腿0，右腿1。从上到下依次0，1，2，3，4
        std::array<std::array<T, human::ARM_MOTOR_NUM>, 2> arm; // 左臂0，右臂1
        // 腰及其他
    };

    using MotorCmd = MotorContainer<iiri::MitMotorCmd>;
    using MotorRet = MotorContainer<iiri::MitMotorRet>;

}; // namespace iiri::human
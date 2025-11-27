/*
 * @Author: 唐文浩
 * @Date: 2025-02-28
 * @LastEditors: 唐文浩
 * @LastEditTime: 2025-06-19
 * @Description:
 *
 * Copyright (c) 2022-2025 by 西湖大学智能产业研究院, All Rights Reserved.
 */
#pragma once
#include "commType.hpp"

namespace iiri::qr
{
    using ImuRet = iiri::ImuRet;

    // 腿顺序(右前，左前，右后，左后)
    // 关节序(abad, hip, knee, wheel)
    struct MotorCmd
    {
        iiri::MitMotorCmd leg[4][4];
    };

    struct MotorRet
    {
        iiri::MitMotorRet leg[4][4];
    };

    struct LinearVelocityData
    {
        double x{0};
        double y{0};
        double z{0};
    };
    using AngularVelocityData = LinearVelocityData;

    struct PoseData
    {
        double roll{0};
        double pitch{0};
        double yaw{0};
    };

    enum class WalkMode
    {
        aiNormal,
        aiClimb,
        aiFast
    };

    // 切换顺序只能是lie->stand->walk->stand->lie，其他顺序均会导致超时错误
    enum class RunState
    {
        lie,       // 趴下
        stand,     // 站立
        walk,      // 行走
        switching, // 切换中，该状态不能设置，在lie-stand之间会出现
    };
} // namespace iiri::qr
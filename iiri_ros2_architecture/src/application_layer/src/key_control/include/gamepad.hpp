/*
 * @Author: 唐文浩
 * @Date: 2024-11-04
 * @LastEditors: 唐文浩
 * @LastEditTime: 2025-01-02
 * @Description: 手柄类
 *
 * Copyright (c) 2022-2024 by 西湖大学智能产业研究院, All Rights Reserved.
 */
#pragma once
#include <cstdint>
#include <optional>
#include <string>

// 手柄返回数据
struct GamepadData
{
    // 按键，按下为1，弹起为0
    int8_t btnA{0};
    int8_t btnB{0};
    int8_t btnX{0};
    int8_t btnY{0};
    int8_t btnLb{0};
    int8_t btnRb{0};
    int8_t btnBack{0};
    int8_t btnStart{0};
    int8_t btnLpress{0}; // 左摇杆按键
    int8_t btnRpress{0}; // 右摇杆按键
    int8_t btnUp{0};
    int8_t btnDown{0};
    int8_t btnLeft{0};
    int8_t btnRight{0};

    double stickLx{0}; // 左摇杆x轴，左-1，右+1
    double stickLy{0}; // 左摇杆y轴，上-1，下+1
    double stickRx{0}; // 右摇杆x轴
    double stickRy{0}; // 右摇杆y轴
    double stickLt{0}; // LT 弹起0，按下+1
    double stickRt{0}; // RT 参数同LT
};

/**
 * @description: 读取手柄信息
 * @return {}
 */
std::optional<GamepadData> GamepadRead(const std::string &devName);

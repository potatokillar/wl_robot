/*
 * @Author: 邹应龙
 * @Date: 2023-10-18
 * @LastEditors: 邹应龙
 * @LastEditTime: 2024-08-23
 * @Description: 四足事件框架，部分情况下，处于架构底层的模块需要传递数据到上层，可通过该接口实现
 *
 * Copyright (c) 2022-2023 by 西湖大学智能产业研究院, All Rights Reserved.
 */
#pragma once
#include <functional>

enum class QrEventType
{
    fall,       // 摔倒事件
    qd_to_big,  // 速度过大
    unstable,   // 不稳定
};

void SetQrEvent(QrEventType type);
void SetQrEventCallback(QrEventType type, std::function<void(QrEventType)> func);
void SetQrEventCallbackBlock(QrEventType type, std::function<void(QrEventType)> func);
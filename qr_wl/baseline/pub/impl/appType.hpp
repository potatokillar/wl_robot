
#pragma once
#include <any>
#include <bitset>
#include <functional>
#include <thread>
#include <vector>

#include "robotType.hpp"

// 机器人状态
enum class RobotState
{
    initing,    // 初始化中，不响应大部分指令
    standby,    // 待机中，算法没起来
    running,    // 运行中，算法已起来
    error,      // 机器人发生错误
    emgstop,    // 急停状态，进入急停状态后只能手动退出
    jointCtrl,  // 关节控制状态，接受关节命令
};

// 通用回调函数
using QrCallbackFunc = std::function<std::any(const std::any &)>;
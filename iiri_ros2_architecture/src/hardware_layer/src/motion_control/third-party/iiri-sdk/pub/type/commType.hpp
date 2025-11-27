/*
 * @Author: 唐文浩
 * @Date: 2025-02-28
 * @LastEditors: 唐文浩
 * @LastEditTime: 2025-06-19
 * @Description: 设备类型，通用类型存放处
 *
 * Copyright (c) 2022-2025 by 西湖大学智能产业研究院, All Rights Reserved.
 */
#pragma once
#include <array>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace iiri
{
    template <typename T>
    constexpr auto Enum2Num(T const value) -> typename std::underlying_type<T>::type
    {
        return static_cast<typename std::underlying_type<T>::type>(value);
    }

    template <typename T>
    struct ValueRange
    {
        T min;
        T max;
        ValueRange() {}
        ValueRange(const T &a, const T &b) : min(a), max(b) {}
    };

    // 返回码，失败返回必须告知失败原因
    enum class RetState
    {
        ok,        // 无错误
        netErr,    // 网络错误
        outRange,  // 超范围错误
        timeout,   // 超时错误
        noSupport, // 功能不被支持错误
        parseErr,  // 协议解析错误
        noUpdate,  // 数据未更新，是旧的数据
        busy,      // 正忙，前一个任务正在执行
        interrupt, // 任务被中断(异常或正常)
        noExist,   // 请求值不存在，注意和noSupport区别
        error,     // 其他错误
    };

    template <typename T>
    using Result = std::pair<RetState, T>;

    // 设备回复信息结构体
    struct DeviceInfo
    {
        std::string name; // 机器名字

        // 版本
        struct Version
        {
            std::string software = "";   // 软件tag版本
            std::string sdk = "";        // sdk版本
            std::string hardware = "";   // 硬件版本
            std::string mechanical = ""; // 结构版本

        } version;
        std::string serialNo; // 序列号
        std::string model;    // 型号
    };

    // 电池有关信息
    struct BatteryInfo
    {
        bool charge{false};   // 是否处于充电中
        uint16_t quantity{0}; // 电量，范围0~100
        float voltage{0};     // 电压 V
        float current{0};     // 电流 A
    };

    // 机器人全局控制
    enum class RobotCtrl
    {
        hold,         // 保持不变
        start,        // 启动控制
        stop,         // 关闭控制
        enterEmgstop, // 进入急停状态
        exitEmgstop,  // 离开急停状态，只有当前模式是急停状态才有用
        lowStart,
    };
    // 机器人全局状态
    enum class RobotState
    {
        initing, // 初始化中，不响应大部分指令
        standby, // 待机中，算法没起来
        running, // 运行中，算法已起来
        error,   // 机器人发生错误
        emgstop, // 急停状态，进入急停状态后只能手动退出
    };

    // mit协议类型的电机命令和返回
    struct MitMotorCmd
    {
        double q{0};
        double qd{0};
        double tau{0};
        double kp{0};
        double kd{0};
    };
    struct MitMotorRet
    {
        double q{0};
        double qd{0};
        double tau{0};
    };

    // imu数据
    struct ImuRet
    {
        // xyz顺序
        double gyro[3]{0};
        double acc[3]{0};
        double ang[3]{0};
        double quat[4]{1, 0, 0, 0}; // wxyz顺序
    };

    enum class RobotEvent
    {
        connect,    // 连接成功
        disconnect, // 断开连接，或连接失败
    };

    // 手柄数据
    struct GamepadCmd
    {
        // 用户一旦操作手柄，即为true；用户不操作手柄一段时间内，为false
        bool takeover{false};

        // 原始值，按键，按下为1，弹起为0
        int a{0};
        int b{0};
        int x{0};
        int y{0};
        int lb{0};
        int rb{0};
        int back{0};
        int start{0};
        int lp{0}; // 左摇杆按键
        int rp{0}; // 右摇杆按键
        int up{0};
        int down{0};
        int left{0};
        int right{0};

        double lx{0}; // 左摇杆x轴，左-1，右+1
        double ly{0}; // 左摇杆y轴，上-1，下+1
        double rx{0}; // 右摇杆x轴
        double ry{0}; // 右摇杆y轴
        double lt{0}; // LT 弹起0，按下+1
        double rt{0}; // RT 参数同LT
    };

} // namespace iiri
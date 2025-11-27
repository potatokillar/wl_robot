/*
 * @Author: 唐文浩
 * @Date: 2024-03-15
 * @LastEditors: 唐文浩
 * @LastEditTime: 2025-02-28
 * @Description: 机械臂的任务级API
 *
 * Copyright (c) 2022-2024 by 西湖大学智能产业研究院, All Rights Reserved.
 */
#pragma once

#include <bitset>
#include <functional>
#include <map>
#include <memory>
#include <string_view>

#include "type/armType.hpp"

namespace iiri
{
    class ApiDevice;
    class SdkProtocol;

    namespace arm
    {
        class ApiArm;
        // 关节位置数据类型

        enum class MoveMode
        {
            abs,  // 绝对
            incr, // 增量
        };

        struct CartesianPose
        {
            double x;
            double y;
            double z;
            double rx;
            double ry;
            double rz;
        };
        struct CartesianPose7 : public CartesianPose
        {
            double phi;
        };

        // 机械臂状态
        enum class ArmState
        {
            noRun,     // 未运行算法
            init,      // 初始化中，不可操作
            noReady,   // 未就绪，需要下发enable让其就绪
            ready,     // 准备就绪，可以下发新任务
            running,   // 正在执行任务中
            pause,     // 任务被暂停，可start重新启动上次任务，也可stop完全放弃该任务
            complete,  // 任务正常完成，一段时间后自动切成ready
            interrupt, // 任务被中断，一段时间后自动切成ready
            error,
        };
        enum class ArmTaskCmd
        {
            start, // 启动/恢复任务，具体是启动还是恢复，是看上一个任务是停止还是暂停
            pause, // 暂停任务，可通过start继续执行
            stop,  // 停止任务，清空未发送任务
        };

        // 机械臂状态信息
        struct ArmInfo
        {
            // 电压，从最底部开始计数
            std::vector<double> voltage;
            // 电流
            std::vector<double> current;
            // 力矩
            std::vector<double> torque;
            // 温度
            std::vector<double> temperature;
        };

        class ArmTask
        {
        public:
            ArmTask(const std::string &ip);
            virtual ~ArmTask();
            ArmTask(const ArmTask &rhs) = delete;
            ArmTask &operator=(const ArmTask &rhs) = delete;
            ArmTask(ArmTask &&rhs);
            ArmTask &operator=(ArmTask &&rhs);

            static const std::string_view &meta();

        public:
            /**
             * @brief 机械臂关节空间规划接口
             *
             * @param jointVal 期望关节角度,可设置多个点(多个点之间会自动平滑)
             * @param moveMode 运动模式：ABS/INCR
             * @param speed 运行绝对值，rad/s。单次有效
             * @param acc 运行加速度，rad/s^2。单次有效
             * @return 阻塞型接口，正常完成或异常中断后均会返回
             */
            RetState SetMoveJTaskBlock(const std::vector<double> &jointVal, MoveMode mode);
            RetState SetMoveJTaskBlock(const std::vector<double> &jointVal, MoveMode mode, double speed);
            RetState SetMoveJTaskBlock(const std::vector<double> &jointVal, MoveMode mode, double speed, double acc);
            RetState SetMoveJTaskBlock(const std::vector<std::vector<double>> &jointVals, MoveMode mode);
            RetState SetMoveJTaskBlock(const std::vector<std::vector<double>> &jointVals, MoveMode mode, double speed);
            RetState SetMoveJTaskBlock(const std::vector<std::vector<double>> &jointVals, MoveMode mode, double speed, double acc);

            /**
             * @brief 机械臂笛卡尔空间规划接口
             *
             * @param posEnd 期望空间位置，可设置多个点(多个点之间会自动平滑)
             * @param moveMode 运动模式：ABS/INCR
             * @param speed 运行绝对值，m/s。单次有效
             * @param acc 运行加速度，m/s^2。单次有效
             * @return 阻塞型接口，正常完成或异常中断后均会返回
             */
            RetState SetMoveLTaskBlock(const CartesianPose &jointVal, MoveMode mode);
            RetState SetMoveLTaskBlock(const CartesianPose &jointVal, MoveMode mode, double speed);
            RetState SetMoveLTaskBlock(const CartesianPose &jointVal, MoveMode mode, double speed, double acc);
            RetState SetMoveLTaskBlock(const std::vector<CartesianPose> &jointVals, MoveMode mode);
            RetState SetMoveLTaskBlock(const std::vector<CartesianPose> &jointVals, MoveMode mode, double speed);
            RetState SetMoveLTaskBlock(const std::vector<CartesianPose> &jointVals, MoveMode mode, double speed, double acc);

            /**
             * @description: 设置机械臂关节空间规划接口，不阻塞。下发后立刻执行
             * 该接口支持多次调用，均会运行。
             * 可通过SetTaskCmd(ArmTaskCmd::stop)或SetTaskCmd(ArmTaskCmd::pause)停止或暂停任务
             * 通过SetTaskCmd(ArmTaskCmd::start)恢复任务
             * @param jointVal 期望关节角度
             * @param moveMode 运动模式：ABS/INCR
             * @param speed 运行绝对值，rad/s。单次有效
             * @param acc 运行加速度，rad/s^2。单次有效
             * @return {}
             */
            RetState SetMoveJTaskNoBlock(const std::vector<double> &jointVal, MoveMode mode);
            RetState SetMoveJTaskNoBlock(const std::vector<double> &jointVal, MoveMode mode, double speed);
            RetState SetMoveJTaskNoBlock(const std::vector<double> &jointVal, MoveMode mode, double speed, double acc);

            /**
             * @description: 设置机械臂笛卡尔空间规划接口，不阻塞。用法同SetMoveJTaskNoBlock
             * @param posEnd 期望空间位置
             * @param moveMode 运动模式：ABS/INCR
             * @param speed 运行绝对值，m/s。单次有效
             * @param acc 运行加速度，m/s^2。单次有效
             * @return {}
             */
            RetState SetMoveLTaskNoBlock(const CartesianPose &jointVal, MoveMode mode);
            RetState SetMoveLTaskNoBlock(const CartesianPose &jointVal, MoveMode mode, double speed);
            RetState SetMoveLTaskNoBlock(const CartesianPose &jointVal, MoveMode mode, double speed, double acc);

            /**
             * @description: 设置机械臂使能状态
             * @param set
             * @return {}
             */
            RetState SetEnable(bool set);

            /**
             * @description: 设置机械臂任务命令
             * @param set
             * @return {}
             */
            RetState SetTaskCmd(ArmTaskCmd cmd);

            /**
             * @description: 获取工具坐标系的名字列表，若算法未启动，则为空
             * @return {}
             */
            Result<std::vector<std::string>> GetToolNameList();

            /**
             * @description: 设置默认工具坐标系
             * @param name
             * @return {}
             */
            RetState SetToolName(const std::string &name);

            /**
             * @description: 获取当前使用的工具坐标系
             * @return {}
             */
            Result<std::string> GetToolName();

            /**
             * @description: 根据工具坐标系名称查询对应的坐标系内容并完成上报
             * @param toolName 需要获取参数的工具坐标系名称
             * @return {} 对应的工具坐标系的内容；若未获取到，则返回非ErrState::ok值
             */
            Result<CartesianPose> GetToolValue(const std::string &name);

            /**
             * @description: 修改对应工具坐标系的具体值
             * @param name 工具坐标系名称
             * @param pose 笛卡尔空间位置
             * @return {}
             */
            RetState SetToolValue(const std::string &name, const CartesianPose &pose);

            /**
             * @description: 正运动学，6轴
             * @param robotjoint 从关节1到关节6的弧度
             * @return {} 笛卡尔空间位置
             */
            Result<CartesianPose> GetFkine(const std::array<double, 6> &robotjoint);

            /**
             * @description: 逆运动学，6轴
             * @param FK 笛卡尔空间位置
             * @return
             */
            Result<std::array<double, 6>> GetIkine(const CartesianPose &FK);

            /**
             * @description: 正运动学，7轴
             * @param robotjoint 从关节1到关节6的弧度
             * @return {} 笛卡尔空间位置
             */
            Result<CartesianPose7> GetFkine7(const std::array<double, 7> &robotjoint);

            /**
             * @description: 逆运动学，7轴
             * @param FK 笛卡尔空间位置
             * @return
             */
            Result<std::array<double, 7>> GetIkine7(const CartesianPose7 &FK);

            /**
             * @description: 获取机械臂当前状态
             * @return {} 机械臂的当前状态，running等
             */
            Result<ArmState> GetArmState();

            /**
             * @description: 订阅机械臂当前状态上报
             * @param func 状态改变时会调用
             * @return {}
             */
            RetState SubscribeArmState(std::function<void(ArmState)> func);

            /**
             * @description: 获取当前关节角
             * @return {} 从关节1到关节6/7的弧度
             */
            Result<std::vector<double>> GetNowAngle();

            /**
             * @description: 订阅关节角上报
             * @param func 定时上报
             * @return {}
             */
            RetState SubscribeNowAngle(std::function<void(const std::vector<double> &)> func);

            /**
             * @description: 获取当前笛卡尔空间位置
             * @return {}
             */
            Result<CartesianPose> GetNowCart();

            /**
             * @description: 订阅笛卡尔空间位置上报
             * @param func 定时上报
             * @return {}
             */
            RetState SubscribeNowCart(std::function<void(const CartesianPose &)> func);

            /**
             * @description: 设置输出io状态
             * @param name io名称，只有panel，tool，extra三种支持
             * @param set 每一个元素代表IO的状态，元素的索引代表IO的序号
             * 对于数字IO，1代表高电平，0代表低电平
             * @return {}
             */
            RetState SetIoOutputState(const std::string &name, std::bitset<32> set);

            /**
             * @description: 设置输出io状态
             * @param name io名称，只有panel，tool，extra三种支持
             * @param pos io位置，从0开始计数
             * @param set
             * @return {}
             */
            RetState SetIoOutputState(const std::string &name, uint8_t pos, bool set);

            /**
             * @description: 获取输出io状态
             * @param name io名称，只有panel，tool，extra三种支持
             * @return {}
             */
            Result<std::bitset<32>> GetIoOutputState(const std::string &name);
            Result<bool> GetIoOutputState(const std::string &name, uint8_t pos);

            /**
             * @description: 获取输入io状态
             * @param name io名称，只有panel，tool，extra三种支持
             * @return {}
             */
            Result<std::bitset<32>> GetIoInputState(const std::string &name);
            Result<bool> GetIoInputState(const std::string &name, uint8_t pos);

            /**
             * @description: 订阅机械臂当前输出IO状态上报
             * @param func 任意一个IO状态改变时会调用
             * @return {}
             */
            RetState SubscribeIoOutputState(std::function<void(const std::map<std::string, std::bitset<32>> &)> func);
            /**
             * @description: 订阅机械臂当前输入IO状态上报
             * @param func 任意一个IO状态改变时会调用
             * @return {}
             */
            RetState SubscribeIoInputState(std::function<void(const std::map<std::string, std::bitset<32>> &)> func);

            /**
             * @description: 获取机械臂限位值
             * @return {}
             */
            Result<std::vector<ValueRange<double>>> GetPositionLimit();

            /**
             * @description: 设置机械臂关节角限位值
             * @param jointVal
             * @return {}
             */
            RetState SetPositionLimit(const std::vector<ValueRange<double>> &jLimit);

            /**
             * @description: 设置机械臂进入运输模式
             * @return {}
             */
            RetState SetToTransport();

            /**
             * @description: 设置机械臂回到零点
             * @return {}
             */
            RetState SetToHome();

            /**
             * @description: 设置机械臂进入拖拽模式
             * @return {}
             */
            RetState SetDragMode(bool set);

            /**
             * @description: 机械臂是否在拖拽模式
             * @return {}
             */
            Result<bool> GetDragMode();

            /**
             * @description: 设置碰撞等级
             * @param level
             * @return {}
             */
            RetState SetCollisionLevel(int level);

            /**
             * @description: 获取碰撞等级
             * @return {}
             */
            Result<int> GetCollisionLevel();

            /**
             * @description: 获取机械臂监控信息
             * @param func
             * @return {}
             */
            Result<ArmInfo> GetArmInfo();

            /**
             * @description: 订阅机械臂监控信息
             * @param func
             * @return {}
             */
            RetState SubscribeArmInfo(std::function<void(const ArmInfo &)> func);

            /**
             * @description: 设置速率
             * @param rate
             * @return {}
             */
            RetState SetRapidrate(double rate);

            /**
             * @description: 获取速率
             * @return {}
             */
            Result<double> GetRapidrate();

            /**
             * @description: 设置关节最大速度
             * @param vMax 最大值，正值。
             * @return {}
             */
            RetState SetJointSpeedLimit(const std::vector<double> &vMax);

            /**
             * @description: 获取关节最大速度
             * @return {}
             */
            Result<std::vector<double>> GetJointSpeedLimit();

            /**
             * @description: 设置关节最大加速度
             * @param aMax 最大值，正值。
             * @return {}
             */
            RetState SetJointAccLimit(const std::vector<double> &aMax);

            /**
             * @description: 获取关节最大加速度
             * @return {}
             */
            Result<std::vector<double>> GetJointAccLimit();

            /**
             * @description: 进入复位模式
             * @param set
             * @return {}
             */
            RetState SetReset(bool set);

            /**
             * @description: 获取复位模式
             * @return {}
             */
            Result<bool> GetReset();
            /**
             * @brief
             * @description: 夹爪控制
             * @param state true是开，false 是关
             * @param speed 0-1
             * @return RetState
             */
            RetState SetGripper(bool state, double speed);

        private:
            std::shared_ptr<SdkProtocol> rpcCli_;
            std::unique_ptr<ApiDevice> apiDev_;
            std::unique_ptr<ApiArm> apiArm_;
        };
    }; // namespace arm
} // namespace iiri

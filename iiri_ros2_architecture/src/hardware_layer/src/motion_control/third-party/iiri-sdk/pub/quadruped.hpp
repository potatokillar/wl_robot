/*
 * @Author: 唐文浩
 * @Date: 2022-06-06
 * @LastEditors: 唐文浩
 * @LastEditTime: 2025-07-16
 * @Description: 四足机器人的API
 *
 * Copyright (c) 2022-2022 by 西湖大学智能产业研究院, All Rights Reserved.
 */

#pragma once

#include <memory>

#include "type/qrType.hpp"

namespace iiri
{
    class ApiDevice;
    class SdkProtocol;

    // 四足有关的均处于qr命名空间
    namespace qr
    {
        class ApiQuadruped;
        class Quadruped
        {
        public:
            Quadruped(const std::string &ip);
            virtual ~Quadruped();
            Quadruped(const Quadruped &rhs) = delete;
            Quadruped &operator=(const Quadruped &rhs) = delete;
            Quadruped(Quadruped &&rhs);
            Quadruped &operator=(Quadruped &&rhs);

            static const std::string_view &meta();

            void SubscribeEvent(std::function<void(RobotEvent evt)> func);

        public:
            /**
             * @description: 设置线速度
             * @param x x方向，m/s。范围可通过GetLinearVelocityRange获取
             * @param y y方向，m/s
             * @param z z方向，m/s。设置无效，内部强制为0
             * @return {} 无需判断返回值，永远返回RetState::ok
             */
            RetState SetLinearVelocity(double x, double y, double z);
            RetState SetLinearVelocity(const LinearVelocityData &v);

            /**
             * @description: 获取线速度，建议使用SubscribeLinearVelocity降低系统占用
             * @param &x
             * @param &y
             * @param &z
             * @return {}
             */
            RetState GetLinearVelocity(double *x, double *y, double *z);
            Result<LinearVelocityData> GetLinearVelocity();

            /**
             * @description: 订阅线速度上报，无论数据是否改变，都以固定频率上报
             * @param func 订阅的回调函数
             * @return {}
             */
            RetState SubscribeLinearVelocity(std::function<void(const LinearVelocityData &)> func);

            /**
             * @description: 获取线速度的取值范围，单位m/s
             * @param &x
             * @param &y
             * @param &z
             * @return {}
             */
            RetState GetLinearVelocityRange(ValueRange<double> *x, ValueRange<double> *y, ValueRange<double> *z);
            std::tuple<RetState, ValueRange<double>, ValueRange<double>, ValueRange<double>> GetLinearVelocityRange();

            /**
             * @description: 设置角速度
             * @param x，无效，内部强制给0
             * @param y，无效，内部强制给0
             * @param z，单位rad/s
             * @return {} 无需判断返回值，永远返回RetState::ok
             */
            RetState SetAngularVelocity(double x, double y, double z);
            RetState SetAngularVelocity(const AngularVelocityData &v);
            /**
             * @description: 获取角速度，建议使用订阅sub函数降低系统占用
             * @param &x
             * @param &y
             * @param &z
             * @return {}
             */
            RetState GetAngularVelocity(double *x, double *y, double *z);
            Result<AngularVelocityData> GetAngularVelocity();
            /**
             * @description: 订阅角速度，无论数据是否改变，都以固定频率上报
             * @param func
             * @return {}
             */
            RetState SubscribeAngularVelocity(std::function<void(const AngularVelocityData &)> func);

            /**
             * @description: 获取角速度的取值范围
             * @param &x
             * @param &y
             * @param &z
             * @return {}
             */
            RetState GetAngularVelocityRange(ValueRange<double> *x, ValueRange<double> *y, ValueRange<double> *z);
            std::tuple<RetState, ValueRange<double>, ValueRange<double>, ValueRange<double>> GetAngularVelocityRange();

            /**
             * @description: 设置姿态
             * @param roll
             * @param pitch
             * @param yaw
             * @return {} 无需判断返回值，永远返回RetState::ok
             */
            RetState SetPose(double roll, double pitch, double yaw);
            RetState SetPose(const PoseData &p);

            RetState GetPose(double *roll, double *pitch, double *yaw);
            Result<PoseData> GetPose();
            /**
             * @description: 订阅姿态角，无论数据是否改变，都以固定频率上报
             * @param func
             * @return {}
             */
            RetState SubscribePose(std::function<void(const PoseData &)> func);

            /**
             * @description: 获取姿态范围
             * @param &roll
             * @param &pitch
             * @param &yaw
             * @return {}
             */
            RetState GetPoseRange(ValueRange<double> *roll, ValueRange<double> *pitch, ValueRange<double> *yaw);
            std::tuple<RetState, ValueRange<double>, ValueRange<double>, ValueRange<double>> GetPoseRange();

            /**
             * @description: 设置高度(相对值)，起立后的高度是0，可以往上正，往下负一定的高度。
             * @param set 单位，m
             * @return {}
             */
            RetState SetHeight(double set);
            Result<double> GetHeight();
            RetState SubscribeHeight(std::function<void(double)> func);

            /**
             * @description: 获取高度设置范围，是默认高度的偏差值
             * @param &height
             * @return {}
             */
            Result<ValueRange<double>> GetHeightRange();

            /**
             * @description: 设置运行状态
             * 运行状态的改变，有可能导致线速度角速度姿态高度上下限的改变，建议重新获取范围
             * @param sta
             * @return {}
             */
            RetState SetRunState(RunState sta);
            Result<RunState> GetRunState();

            /**
             * @description: 订阅运行状态的变化，该状态只有在改变时才会上报。
             * 建议使用该函数前，先用一次GetRunState()获取当前状态保存
             * @param func
             * @return {}
             */
            RetState SubscribeRunState(std::function<void(RunState)> func);

            /**
             * @description: 获取运行模式的取值范围
             * @param &state
             * @return {}
             */
            Result<std::vector<std::string>> GetRunStateRange();

            /**
             * @description: 设置步行模式
             * 步行模式的改变，有可能导致线速度角速度姿态高度上下限的改变，建议重新获取
             * @param mode
             * @return {}
             */
            RetState SetWalkMode(WalkMode mode);

            /**
             * @description: 获取当前步行模式
             * @param &mode
             * @return {}
             */
            Result<WalkMode> GetWalkMode();

            /**
             * @description: 订阅步行模式的状态变化，该值只有在改变时才上报
             * 建议使用该函数前，先用一次GetWalkMode()获取当前状态保存
             * @param func
             * @return {}
             */
            RetState SubscribeWalkMode(std::function<void(WalkMode)> func);

            /**
             * @description: 获取步行状态的取值范围
             * @param &mode
             * @return {}
             */
            Result<std::vector<std::string>> GetWalkModeRange();

            /**
             * @description: 获取电池信息，例如电量，状态等
             * @param &info
             * @return {}
             */
            Result<BatteryInfo> GetBatteryInfo();

            /**
             * @description: 订阅电池信息，该值默认以默认间隔或状态改变上报，具体逻辑同实际机器人有关
             * 通常，是定时上报；但出现若充电状态改变等状态，会立即上报。
             * @param func
             * @return {}
             */
            RetState SubscribeBatteryInfo(std::function<void(const BatteryInfo &)> func);

            /**
             * @description: 获取设备信息
             * @param &info
             * @return {}
             */
            Result<DeviceInfo> GetDeviceInfo();

            /**
             * @description: 设置紧急停止
             * @param set true-开启紧急停止，false-关闭紧急停止
             * @return {}
             */
            RetState SetEmergStop(bool set);
            Result<bool> GetEmergStop();

            /**
             * @description: 获取机器人全局当前状态
             * @return {}
             */
            Result<RobotState> GetRobotState();

            /**
             * @description: 订阅机器人状态，机器人状态改变会上报
             * 因此建议使用该函数前，先用GetRobotState()获取一次状态
             * @param func
             * @return {}
             */
            RetState SubscribetRobotState(std::function<void(RobotState)> func);

            /**
             * @description: 设置负载质量，负载质量的改变会导致机器人运动状态的调整
             * 该负载质量是不包含机器人本地的额外重量
             * @param set，单位kg
             * @return {}
             */
            RetState SetLoadMass(double set);

            /**
             * @description: 获取当前已设置的负载质量
             * @return {}
             */
            Result<double> GetLoadMass();

            /**
             * @description: 获取参数的设置范围
             * @return {}
             */
            Result<ValueRange<double>> GetLoadMassRange();

            /**
             * @description: 获取原始IMU数据
             * @param *data
             * @return {}
             */
            Result<qr::ImuRet> GetImuData();

            /**
             * @description: 订阅imu原始数据上报，不管数据是否被改变，都会已固定的频率进行上报
             * @param func
             * @return {}
             */
            RetState SubscribetImuData(std::function<void(const ImuRet &)> func);

            /**
             * @description: 设置跳舞，目前仅stand模式下支持
             * 注意，该函数是阻塞型函数
             * @param set 舞蹈名字，目前是0~3
             * @param ms 超时时间
             * @return {}
             */
            RetState SetDance(const std::string &name, uint32_t ms = 20000);

            /**
             * @description: 订阅手柄数据上报
             * @param func
             * @return {}
             */
            RetState SubscribeGamepadCmd(std::function<void(const GamepadCmd &)> func);

            /**
             * @description: 设置灯条为一种固定颜色。颜色由RGB三个颜色值给出，颜色值的范围是0-255
             */
            RetState SetRgbLedToSolidColor(uint8_t red, uint8_t green, uint8_t blue);

            /**
             * @description: 灯条颜色在2个颜色之间反复变化，两个颜色分别由RGB颜色值给出。
             * @param cycle 颜色变化的时长，单位是100 ms。
             */
            RetState SetRgbLedToBreathing(uint8_t redStart, uint8_t greenStart, uint8_t blueStart, uint8_t redEnd, uint8_t greenEnd,
                                          uint8_t blueEnd, uint8_t cycle);

            /**
             * @description: 灯条进入呼吸灯模式，颜色固定为红、绿、蓝三色，颜色不可更换，但是可以更换循环时长和亮度
             * @param autoCycle 颜色变化的时长，单位是100 ms。
             * @param autoBrightnessPercentage 亮度，从0-100的整数值，100为最大亮度，0为不亮。
             */
            RetState SetRgbLedToAutoBreathing(uint8_t autoCycle, uint8_t autoBrightnessPercentage);

        private:
            std::shared_ptr<SdkProtocol> rpcCli_;
            std::unique_ptr<ApiDevice> apiDev_;
            std::unique_ptr<ApiQuadruped> apiQuad_;
        };
    }; // namespace qr

} // namespace iiri
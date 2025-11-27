/*
 * @Author: 唐文浩
 * @Date: 2025-06-17
 * @LastEditors: 唐文浩
 * @LastEditTime: 2025-06-17
 * @Description: 四足机器人通信控制模块
 *
 * Copyright (c) 2022-2025 by 西湖大学智能产业研究院, All Rights Reserved.
 */
#pragma once

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>

#include "interface/msg/battery_info.hpp"
#include "interface/msg/gamepad_cmd.hpp"
#include "interface/msg/pose.hpp"
#include "interface/msg/quad_state.hpp"
#include "interface/msg/robot_state.hpp"
#include "interface/msg/run_state.hpp"
#include "interface/msg/walk_mode.hpp"
#include "interface/srv/dance.hpp"
#include "interface/srv/device_info.hpp"
#include "interface/srv/dog_move.hpp"
#include "quadruped.hpp"
#include "robot_base/timer_tools.hpp"

class qr_node : public rclcpp::Node
{
public:
    qr_node();

private:
    void Run();
    void sdk_connect();

    // 接收其他node的指令
    void RxVel(const geometry_msgs::msg::Twist::SharedPtr msg);
    void RxVelRatio(const geometry_msgs::msg::Twist::SharedPtr msg);
    void RxPos(const interface::msg::Pose::SharedPtr msg);
    void RxPosRatio(const interface::msg::Pose::SharedPtr msg);
    void RxQuadState(const interface::msg::QuadState::SharedPtr msg);
    void RxWalkMode(const interface::msg::WalkMode::SharedPtr msg);
    void RxSetRunState(const interface::msg::RunState::SharedPtr msg);
    void RxDeviceInfo(const std::shared_ptr<interface::srv::DeviceInfo::Request> request, std::shared_ptr<interface::srv::DeviceInfo::Response> response);
    void RxDogMove(const std::shared_ptr<interface::srv::DogMove::Request> request, std::shared_ptr<interface::srv::DogMove::Response> response);
    void RxDance(const std::shared_ptr<interface::srv::Dance::Request> request, std::shared_ptr<interface::srv::Dance::Response> response);
    void RxHeight(const std_msgs::msg::Float32::SharedPtr msg);
    void RxEmergStop(const std_msgs::msg::Bool::SharedPtr msg);

    // 发送到其他node
    void PubRunState(const iiri::qr::RunState &state);
    void PubWalkMode(const iiri::qr::WalkMode &mode);
    void PubBatteryInfo(const iiri::BatteryInfo &info);
    void PubPose(const iiri::qr::PoseData &pose);
    void PubHeight(const float &height);
    void PubLinearVelocity(const iiri::qr::LinearVelocityData &velocity);
    void PubAngularVelocity(const iiri::qr::AngularVelocityData &velocity);
    void PubGamepadCmd(const iiri::GamepadCmd &cmd);

private:
    std::unique_ptr<iiri::qr::Quadruped> quadruped_;
    iiri::qr::RunState runState_;
    iiri::qr::WalkMode walkMode_;
    geometry_msgs::msg::Twist nowVel_;
    interface::msg::QuadState nowQuadState_;

private:
    // ros接口
    rclcpp::TimerBase::SharedPtr timer_recv_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_vel_;       // 线速度角速度指令
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_vel_ratio_; // 线速度角速度比例
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_vel_;          // 线速度角速度返回

    rclcpp::Subscription<interface::msg::Pose>::SharedPtr sub_pos_; // 姿态指令
    rclcpp::Subscription<interface::msg::Pose>::SharedPtr sub_pos_ratio_;
    rclcpp::Publisher<interface::msg::Pose>::SharedPtr pub_pos_; // 姿态返回

    rclcpp::Subscription<interface::msg::RobotState>::SharedPtr sub_state_; // 机器人状态
    rclcpp::Publisher<interface::msg::RobotState>::SharedPtr pub_state_;

    rclcpp::Subscription<interface::msg::QuadState>::SharedPtr sub_quad_state_; // 四足状态
    rclcpp::Publisher<interface::msg::QuadState>::SharedPtr pub_quad_state_;

    rclcpp::Subscription<interface::msg::RunState>::SharedPtr sub_run_state_; // 行走状态
    rclcpp::Publisher<interface::msg::RunState>::SharedPtr pub_run_state_;

    rclcpp::Subscription<interface::msg::WalkMode>::SharedPtr sub_walk_mode_; // 行走模式
    rclcpp::Publisher<interface::msg::WalkMode>::SharedPtr pub_walk_mode_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_emerg_stop_; // 收听紧急停止指令
    rclcpp::Publisher<interface::msg::BatteryInfo>::SharedPtr pub_battery_info_;

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_height_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_height_;

    rclcpp::Publisher<interface::msg::GamepadCmd>::SharedPtr pub_gamepad_cmd_; // 发布手柄指令

    // 设备信息
    rclcpp::Service<interface::srv::DeviceInfo>::SharedPtr srv_dev_info_;
    rclcpp::Service<interface::srv::DogMove>::SharedPtr srv_dog_move_;
    rclcpp::Service<interface::srv::Dance>::SharedPtr srv_dance_;
};
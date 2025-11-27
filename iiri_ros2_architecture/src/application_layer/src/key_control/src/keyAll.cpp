/*
 * @Author: 唐文浩
 * @Date: 2024-11-06
 * @LastEditors: 唐文浩
 * @LastEditTime: 2025-03-06
 * @Description: 四足和机械臂联合控制
 *
 * Copyright (c) 2022-2024 by 西湖大学智能产业研究院, All Rights Reserved.
 */
#include <std_msgs/msg/float32.hpp>
#include <thread>

#include "gamepad.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "interface/msg/arm_state.hpp"
#include "interface/msg/battery_info.hpp"
#include "interface/msg/pose.hpp"
#include "interface/msg/quad_state.hpp"
#include "interface/msg/robot_state.hpp"
#include "interface/msg/run_state.hpp"
#include "interface/msg/walk_mode.hpp"
#include "interface/srv/arm_cmd.hpp"
#include "interface/srv/device_info.hpp"
#include "interface/srv/move_j.hpp"
#include "interface/srv/move_l.hpp"
#include "interface/srv/set_gripper.hpp"
#include "keyBase.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_base/timer_tools.hpp"

// 四足按键逻辑
class KeyQr
{
public:
    KeyQr(std::shared_ptr<rclcpp::Node> node) : node_(node)
    {
        js_ = node_->get_parameter("js").as_string();
        RCLCPP_INFO(node_->get_logger(), "js: %s", js_.c_str());
        thread_ = std::thread(&KeyQr::Loop, this);

        pub_run_state_ = node_->create_publisher<interface::msg::RunState>("/motion_control/cmd_run_state", 10);
        pub_pos_ = node_->create_publisher<interface::msg::Pose>("/motion_control/cmd_pose", 10);
        pub_vel_ = node_->create_publisher<geometry_msgs::msg::Twist>("/motion_control/cmd_vel", 10);
        pub_height_ = node_->create_publisher<std_msgs::msg::Float32>("/motion_control/cmd_height", 10);

        sub_height_ = node_->create_subscription<std_msgs::msg::Float32>("/motion_control/ret_height", 10, [this](const std_msgs::msg::Float32::SharedPtr msg)
                                                                         { this->height_.data = msg->data; });
    }

private:
    void Loop()
    {
        while (true)
        {
            TimerTools::SleepForMs(10); // 降低CPU占用

            auto jsData = GamepadRead(js_);
            if (jsData.has_value())
            {
                key_.SaveGamepadBtn(jsData.value());
            }
            else
            {
                // 无按键输入，暂不做处理
            }

            //  读取数据
            if (key_.IsShortRelease("a"))
            {
                RCLCPP_INFO(node_->get_logger(), "btnA press");
                interface::msg::RunState sta;
                sta.value = interface::msg::RunState::STAND;
                pub_run_state_->publish(sta);
            }
            if (key_.IsShortRelease("b"))
            {
                RCLCPP_INFO(node_->get_logger(), "btnB press");
                interface::msg::RunState sta;
                sta.value = interface::msg::RunState::WALK;
                pub_run_state_->publish(sta);
            }
            if (key_.IsShortRelease("x"))
            {
                RCLCPP_INFO(node_->get_logger(), "btnX press");
                interface::msg::RunState sta;
                sta.value = interface::msg::RunState::LIE;
                pub_run_state_->publish(sta);
            }

            // 运动
            if (run_sta_.value == interface::msg::RunState::STAND)
            {
                // ly 高度
                std_msgs::msg::Float32 height;
                height.data = key_.GetStick().at("ly") * 0.05 + height_.data;
                pub_height_->publish(height);

                interface::msg::Pose pos;
                pos.roll = key_.GetStick().at("lx") * 0.3;
                pos.pitch = key_.GetStick().at("ry") * 0.3;
                pos.yaw = key_.GetStick().at("rx") * 0.3;
                pub_pos_->publish(pos);
            }
            else if (run_sta_.value == interface::msg::RunState::WALK)
            {
                geometry_msgs::msg::Twist vel;
                vel.linear.x = key_.GetStick().at("ly") * -1.0;
                vel.linear.y = key_.GetStick().at("lx") * -0.6;
                vel.angular.z = key_.GetStick().at("rx") * -1.5;
                pub_vel_->publish(vel);
            }
        }
    }

private:
    std::shared_ptr<rclcpp::Node> node_;
    std::thread thread_;
    std::string js_;
    KeyBase key_;

    // 旧数据
    geometry_msgs::msg::Twist vel_;
    interface::msg::Pose pos_;
    std_msgs::msg::Float32 height_;
    interface::msg::QuadState qr_sta_;
    interface::msg::RunState run_sta_;
    interface::msg::WalkMode walk_mode_;
    interface::msg::BatteryInfo battery_info_;
    interface::msg::RobotState robot_state_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_vel_; // 线速度角速度
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_vel_;

    rclcpp::Subscription<interface::msg::Pose>::SharedPtr sub_pos_; // 姿态
    rclcpp::Publisher<interface::msg::Pose>::SharedPtr pub_pos_;

    rclcpp::Subscription<interface::msg::QuadState>::SharedPtr sub_qr_state_; // 四足特有状态
    rclcpp::Publisher<interface::msg::QuadState>::SharedPtr pub_qr_state_;

    rclcpp::Subscription<interface::msg::RunState>::SharedPtr sub_run_state_;
    rclcpp::Publisher<interface::msg::RunState>::SharedPtr pub_run_state_;

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_height_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_height_;

    rclcpp::Subscription<interface::msg::WalkMode>::SharedPtr sub_walk_mode_;
    rclcpp::Publisher<interface::msg::WalkMode>::SharedPtr pub_walk_mode_;
};

// 机械臂按键逻辑
class KeyArm
{
public:
    KeyArm(std::shared_ptr<rclcpp::Node> node) : node_(node)
    {
        js_ = node_->get_parameter("js").as_string();
        RCLCPP_INFO(node_->get_logger(), "js: %s", js_.c_str());
        thread_ = std::thread(&KeyArm::Loop, this);

        cli_cmd_ = node_->create_client<interface::srv::ArmCmd>("/motion_control/arm_cmd");
        cli_move_j_ = node_->create_client<interface::srv::MoveJ>("/motion_control/move_j");
        cli_move_l_ = node_->create_client<interface::srv::MoveL>("/motion_control/move_l");
        cli_gripper_ = node_->create_client<interface::srv::SetGripper>("/motion_control/set_gripper");

        sub_state_ =
            node_->create_subscription<interface::msg::ArmState>("/motion_control/arm_state", 10, [this](const interface::msg::ArmState::SharedPtr msg)
                                                                 {
                now_state_.cart = msg->cart;
                now_state_.angle = msg->angle;
                now_state_.state = msg->state;
                RCLCPP_INFO(node_->get_logger(), "arm state: %f", msg->angle[0]); });
    }

private:
    void Loop()
    {
        while (true)
        {
            auto jsData = GamepadRead(js_);
            if (jsData.has_value())
            {
                key_.SaveGamepadBtn(jsData.value());
            }
            else
            {
                // 无按键输入，暂不做处理
            }
            {
                bool is_ok = false;
                auto request = std::make_shared<interface::srv::ArmCmd::Request>();
                if (key_.IsShortRelease("start"))
                {
                    request->cmd = interface::srv::ArmCmd::Request::ENABLE;
                    is_ok = true;
                }
                if (key_.IsShortRelease("back"))
                {
                    request->cmd = interface::srv::ArmCmd::Request::DISABLE;
                    is_ok = true;
                }

                if (is_ok)
                {
                    cli_cmd_->async_send_request(request, [this](rclcpp::Client<interface::srv::ArmCmd>::SharedFuture future)
                                                 {
                        auto response = future.get();
                        RCLCPP_INFO(node_->get_logger(), "enable: %d", response->result); });
                }
            }

            if (key_.IsLongPress("rt"))
            {
                auto request = std::make_shared<interface::srv::MoveJ::Request>();
                for (int i = 0; i < 6; i++)
                {
                    request->joint.at(i) = now_state_.angle[i];
                }
                bool is_press = false;
                if (key_.GetStick().at("rx") > 0.5)
                {
                    request->joint.at(0) += 0.1;
                    is_press = true;
                }
                if (key_.GetStick().at("rx") < -0.5)
                {
                    request->joint.at(0) -= 0.1;
                    is_press = true;
                }

                if (key_.GetStick().at("ry") > 0.5)
                {
                    request->joint.at(1) -= 0.1;
                    is_press = true;
                }
                if (key_.GetStick().at("ry") < -0.5)
                {
                    request->joint.at(1) += 0.1;
                    is_press = true;
                }

                if (key_.GetStick().at("ly") > 0.5)
                {
                    request->joint.at(2) += 0.3;
                    is_press = true;
                }
                if (key_.GetStick().at("ly") < -0.5)
                {
                    request->joint.at(2) -= 0.3;
                    is_press = true;
                }

                if (key_.GetStick().at("lx") > 0.5)
                {
                    request->joint.at(3) += 0.3;
                    is_press = true;
                }
                if (key_.GetStick().at("lx") < -0.5)
                {
                    request->joint.at(3) -= 0.3;
                    is_press = true;
                }

                if (key_.IsShortRelease("up"))
                {
                    request->joint.at(4) -= 0.3;
                    is_press = true;
                }
                if (key_.IsShortRelease("down"))
                {
                    request->joint.at(4) += 0.3;
                    is_press = true;
                }

                if (key_.IsShortRelease("left"))
                {
                    request->joint.at(5) -= 0.3;
                    is_press = true;
                }
                if (key_.IsShortRelease("right"))
                {
                    request->joint.at(5) += 0.3;
                    is_press = true;
                }

                if (is_press)
                {
                    request->mode = "abs";
                    request->speed = 0;
                    request->acc = 0;
                    auto future = cli_move_j_->async_send_request(request);
                    if (future.get()->result == 0)
                    {
                        RCLCPP_INFO(node_->get_logger(), "SUCCESS!!! movej joint");
                    }
                    else
                    {
                        RCLCPP_WARN(node_->get_logger(), "FAILURE!!! movej joint");
                    }
                }
                auto reqGripper = std::make_shared<interface::srv::SetGripper::Request>();
                if (key_.IsLongPress("lb"))
                {
                    reqGripper->state = true;
                    reqGripper->speed = 1;
                    auto future = cli_gripper_->async_send_request(reqGripper);
                    if (future.get()->result == 0)
                    {
                        RCLCPP_INFO(node_->get_logger(), "SUCCESS!!! set gripper");
                    }
                    else
                    {
                        RCLCPP_WARN(node_->get_logger(), "FAILURE!!! set gripper");
                    }
                }
                else if (key_.IsLongPress("lt"))
                {
                    reqGripper->state = false;
                    reqGripper->speed = 1;
                    auto future = cli_gripper_->async_send_request(reqGripper);
                    if (future.get()->result == 0)
                    {
                        RCLCPP_INFO(node_->get_logger(), "SUCCESS!!! set gripper");
                    }
                    else
                    {
                        RCLCPP_WARN(node_->get_logger(), "FAILURE!!! set gripper");
                    }
                }
            }
        }
    }

private:
    std::shared_ptr<rclcpp::Node> node_;
    std::thread thread_;
    std::string js_;
    KeyBase key_;

    rclcpp::Client<interface::srv::ArmCmd>::SharedPtr cli_cmd_;
    rclcpp::Client<interface::srv::MoveJ>::SharedPtr cli_move_j_;
    rclcpp::Client<interface::srv::MoveL>::SharedPtr cli_move_l_;
    rclcpp::Subscription<interface::msg::ArmState>::SharedPtr sub_state_;
    rclcpp::Client<interface::srv::SetGripper>::SharedPtr cli_gripper_;

    interface::msg::ArmState now_state_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("key_all_node");
    node->declare_parameter<std::string>("js", "/dev/input/js0");
    RCLCPP_INFO(node->get_logger(), "Hello key all!");
    KeyQr keyQr(node);
    KeyArm keyArm(node);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
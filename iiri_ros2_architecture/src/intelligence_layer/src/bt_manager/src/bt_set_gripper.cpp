/*
 * @Author: 唐文浩
 * @Date: 2025-01-10
 * @LastEditors: 唐文浩
 * @LastEditTime: 2025-02-06
 * @Description:
 *
 * Copyright (c) 2022-2025 by 西湖大学智能产业研究院, All Rights Reserved.
 */
#include "bt_set_gripper.hpp"

#include "robot_base/timer_tools.hpp"
using namespace BT;
using namespace std;

// moveJ
BtSetGripper::BtSetGripper(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node)
    : ThreadedAction(name, config), node_(node)
{
    cli_set_gripper_ = node_->create_client<interface::srv::SetGripper>("/motion_control/set_gripper");
}

NodeStatus BtSetGripper::tick()
{
    // 目标点必须给，其他的可以是默认
    bool state;
    if (getInput("state", state))
    {
        getInput("speed", speed_);

        auto request = std::make_shared<interface::srv::SetGripper::Request>();
        request->state = state;
        request->speed = speed_;

        auto result_future = cli_set_gripper_->async_send_request(request);
        TimerTools::SleepForS(1);
        if (result_future.get()->result == 0)
        {
            RCLCPP_INFO(node_->get_logger(), "SUCCESS!!! set gripper:%d; spd:%f", state, speed_);
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            RCLCPP_INFO(node_->get_logger(), "FAILURE!!! set gripper:%d; spd:%f", state, speed_);
            return BT::NodeStatus::FAILURE;
        }
    }

    return BT::NodeStatus::FAILURE;
}

BT::PortsList BtSetGripper::providedPorts() { return {BT::InputPort<bool>("state"), BT::InputPort<double>("speed")}; }

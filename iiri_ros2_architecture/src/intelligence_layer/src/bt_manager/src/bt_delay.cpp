/*
 * @Author: 唐文浩
 * @Date: 2025-01-10
 * @LastEditors: 唐文浩
 * @LastEditTime: 2025-02-06
 * @Description:
 *
 * Copyright (c) 2022-2025 by 西湖大学智能产业研究院, All Rights Reserved.
 */
#include "bt_delay.hpp"

#include "robot_base/timer_tools.hpp"

using namespace BT;
using namespace std;

// moveJ
BtDelay::BtDelay(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node) : ThreadedAction(name, config), node_(node) {}

NodeStatus BtDelay::tick()
{
    // 目标点必须给，其他的可以是默认
    double sec;
    if (getInput("sec", sec))
    {
        TimerTools::SleepForMs(sec * 1000);
        return BT::NodeStatus::SUCCESS;
    }
    RCLCPP_ERROR(node_->get_logger(), "must input sec");
    return BT::NodeStatus::FAILURE;
}

BT::PortsList BtDelay::providedPorts() { return {BT::InputPort<double>("sec")}; }

/*
 * @Author: 唐文浩
 * @Date: 2025-01-08
 * @LastEditors: 唐文浩
 * @LastEditTime: 2025-01-13
 * @Description:
 *
 * Copyright (c) 2022-2025 by 西湖大学智能产业研究院, All Rights Reserved.
 */
#pragma once

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

// 智能跟随检测节点（异步执行）
class BtSmartFollow : public BT::ThreadedAction
{
public:
    BtSmartFollow(const std::string &name, const BT::NodeConfig &config);
    BtSmartFollow(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node);
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts();

private:
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cli_trigger_;
    bool service_available_ = true;
};
/*
 * @Author: 唐文浩
 * @Date: 2025-01-21
 * @LastEditors: 唐文浩
 * @LastEditTime: 2025-01-21
 * @Description: 远程控制节点
 *
 * Copyright (c) 2022-2025 by 西湖大学智能产业研究院, All Rights Reserved.
 */
#pragma once

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

// 远程控制检测节点（异步执行）
class BtRemoteCtrl : public BT::ThreadedAction
{
public:
    BtRemoteCtrl(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node);
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts();

private:
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cli_trigger_;
    bool service_available_ = true;
};
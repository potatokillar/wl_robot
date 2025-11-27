/*
 * @Author: 唐文浩
 * @Date: 2025-01-08
 * @LastEditors: 唐文浩
 * @LastEditTime: 2025-01-14
 * @Description:
 *
 * Copyright (c) 2022-2025 by 西湖大学智能产业研究院, All Rights Reserved.
 */
#pragma once

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

// 没有延迟的节点
class BtKeyCtrl : public BT::ThreadedAction
{
public:
    BtKeyCtrl(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node);
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts();

private:
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cli_trigger_;
};
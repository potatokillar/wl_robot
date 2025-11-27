/*
 * @Author: 唐文浩
 * @Date: 2025-01-17
 * @LastEditors: 唐文浩
 * @LastEditTime: 2025-01-17
 * @Description:
 *
 * Copyright (c) 2022-2025 by 西湖大学智能产业研究院, All Rights Reserved.
 */

#pragma once
#include <behaviortree_cpp/action_node.h>

#include <array>
#include <memory>
#include <rclcpp/rclcpp.hpp>

class BtDelay : public BT::ThreadedAction
{
public:
    BtDelay(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node);
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts();

private:
    std::shared_ptr<rclcpp::Node> node_;
};
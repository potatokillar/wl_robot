/*
 * @Author: 唐文浩
 * @Date: 2025-01-09
 * @LastEditors: 唐文浩
 * @LastEditTime: 2025-01-17
 * @Description: 机械臂任务行为树
 *
 * Copyright (c) 2022-2025 by 西湖大学智能产业研究院, All Rights Reserved.
 */
#pragma once

#include <array>
#include <memory>

#include "behaviortree_cpp/action_node.h"
#include "interface/msg/arm_state.hpp"
#include "interface/srv/arm_cmd.hpp"
#include "interface/srv/move_j.hpp"
#include "interface/srv/move_l.hpp"
#include "rclcpp/rclcpp.hpp"

class BtArmMoveJ : public BT::ThreadedAction
{
public:
    BtArmMoveJ(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node);
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts();

private:
    std::shared_ptr<rclcpp::Node> node_;
    std::string mode_ = "abs";
    double speed_ = 0.2;
    double acc_ = 0.2;

    rclcpp::Client<interface::srv::MoveJ>::SharedPtr cli_move_j_;
};

class BtArmMoveL : public BT::ThreadedAction
{
public:
    BtArmMoveL(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node);
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts();

private:
    std::shared_ptr<rclcpp::Node> node_;
    std::string mode_ = "abs";
    double speed_ = 0.2;
    double acc_ = 0.2;

    void RxArmState(const interface::msg::ArmState::SharedPtr msg);

    rclcpp::Client<interface::srv::MoveL>::SharedPtr cli_move_l_;
    rclcpp::Subscription<interface::msg::ArmState>::SharedPtr sub_arm_state_;
};

// 没有延迟的节点
class BtArmEnable : public BT::SyncActionNode
{
public:
    BtArmEnable(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node);
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts();

private:
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Client<interface::srv::ArmCmd>::SharedPtr cli_cmd_;
};
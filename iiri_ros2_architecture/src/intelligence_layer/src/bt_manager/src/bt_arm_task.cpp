/*
 * @Author: 唐文浩
 * @Date: 2025-01-10
 * @LastEditors: 唐文浩
 * @LastEditTime: 2025-02-06
 * @Description:
 *
 * Copyright (c) 2022-2025 by 西湖大学智能产业研究院, All Rights Reserved.
 */
#include "bt_arm_task.hpp"

using namespace BT;
using namespace std;

// moveJ
BtArmMoveJ::BtArmMoveJ(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node) : ThreadedAction(name, config), node_(node)
{
    cli_move_j_ = node_->create_client<interface::srv::MoveJ>("/motion_control/move_j");
}

NodeStatus BtArmMoveJ::tick()
{
    // 目标点必须给，其他的可以是默认
    std::vector<double> joint;
    if (getInput("joint", joint))
    {
        if (joint.size() != 7 && joint.size() != 6)
        {
            return BT::NodeStatus::FAILURE;
        }
        getInput("mode", mode_);
        getInput("speed", speed_);
        getInput("acc", acc_);

        auto targetj = std::make_shared<interface::srv::MoveJ::Request>();
        for (size_t i = 0; i < joint.size(); i++)
        {
            targetj->joint.at(i) = joint.at(i);
        }
        targetj->mode = mode_;
        targetj->speed = speed_;
        targetj->acc = acc_;
        auto result_future = cli_move_j_->async_send_request(targetj);
        if (result_future.get()->result == 0)
        {
            RCLCPP_INFO(node_->get_logger(),
                        "SUCCESS!!! movej joint:%f, %f, %f, %f, %f, %f; spd:%f, acc:%f",
                        joint.at(0),
                        joint.at(1),
                        joint.at(2),
                        joint.at(3),
                        joint.at(4),
                        joint.at(5),
                        speed_,
                        acc_);
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            RCLCPP_WARN(node_->get_logger(),
                        "FAILURE! movej joint:%f, %f, %f, %f, %f, %f; spd:%f, acc:%f",
                        joint.at(0),
                        joint.at(1),
                        joint.at(2),
                        joint.at(3),
                        joint.at(4),
                        joint.at(5),
                        speed_,
                        acc_);
            return BT::NodeStatus::FAILURE;
        }
    }

    return BT::NodeStatus::FAILURE;
}

BT::PortsList BtArmMoveJ::providedPorts()
{
    return {BT::InputPort<std::vector<double>>("joint"), BT::InputPort<std::string>("mode"), BT::InputPort<double>("speed"), BT::InputPort<double>("acc")};
}

//////////////////////////move l////////////////////
BtArmMoveL::BtArmMoveL(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node) : ThreadedAction(name, config), node_(node)
{
    cli_move_l_ = node_->create_client<interface::srv::MoveL>("/motion_control/move_l");
    sub_arm_state_ = node_->create_subscription<interface::msg::ArmState>("/motion_control/arm_state",
                                                                          10,
                                                                          [this](const interface::msg::ArmState::SharedPtr msg)
                                                                          { this->RxArmState(msg); });
}

void BtArmMoveL::RxArmState(const interface::msg::ArmState::SharedPtr msg)
{
    (void)msg;
    // RCLCPP_INFO(node_->get_logger(),
    //     "angle: %lf, %lf, %lf, %lf, %lf, %lf, %lf",
    //   msg->cart.x,
    //  msg->cart.y,
    //  msg->cart.z,
    //  msg->cart.roll,
    // msg->cart.pitch,
    //  msg->cart.yaw);
}

NodeStatus BtArmMoveL::tick()
{
    string name;
    getInput("name", name);
    RCLCPP_INFO(node_->get_logger(), "name:%s", name.c_str());
    // 目标点必须给，其他的可以是默认
    std::vector<double> target;
    if (getInput("target", target))
    {
        getInput("mode", mode_);
        getInput("speed", speed_);
        getInput("acc", acc_);

        if (target.size() != 6)
        {
            return BT::NodeStatus::FAILURE;
        }

        {
        }

        auto request = std::make_shared<interface::srv::MoveL::Request>();
        request->tran.x = target.at(0);
        request->tran.y = target.at(1);
        request->tran.z = target.at(2);

        request->rpy.x = target.at(3);
        request->rpy.y = target.at(4);
        request->rpy.z = target.at(5);

        request->mode = mode_;
        request->speed = speed_;
        request->acc = acc_;
        auto ret_future = cli_move_l_->async_send_request(request);
        if (ret_future.get()->result == 0)
        {
            RCLCPP_INFO(node_->get_logger(),
                        "SUCCESS!!! moveL target:%f, %f, %f, %f, %f, %f; spd:%f, acc:%f",
                        target.at(0),
                        target.at(1),
                        target.at(2),
                        target.at(3),
                        target.at(4),
                        target.at(5),
                        speed_,
                        acc_);
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            RCLCPP_WARN(node_->get_logger(),
                        "FAILURE!!! moveL target:%f, %f, %f, %f, %f, %f; spd:%f, acc:%f",
                        target.at(0),
                        target.at(1),
                        target.at(2),
                        target.at(3),
                        target.at(4),
                        target.at(5),
                        speed_,
                        acc_);
            return BT::NodeStatus::FAILURE;
        }
    }
    return BT::NodeStatus::FAILURE;
}

BT::PortsList BtArmMoveL::providedPorts()
{
    return {BT::InputPort<std::vector<double>>("target"), BT::InputPort<std::string>("mode"), BT::InputPort<double>("speed"), BT::InputPort<double>("acc")};
}

//////////////////////// 使能接口 //////////////////////////
BtArmEnable::BtArmEnable(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node) : SyncActionNode(name, config), node_(node)
{
    cli_cmd_ = node_->create_client<interface::srv::ArmCmd>("/motion_control/arm_cmd");
}

NodeStatus BtArmEnable::tick()
{
    // 目标点必须给，其他的可以是默认
    bool enable;
    if (getInput("enable", enable))
    {
        auto request = std::make_shared<interface::srv::ArmCmd::Request>();
        if (enable)
        {
            request->cmd = interface::srv::ArmCmd::Request::ENABLE;
        }
        else
        {
            request->cmd = interface::srv::ArmCmd::Request::DISABLE;
        }

        auto ret_future = cli_cmd_->async_send_request(request);
        if (ret_future.get()->result == 0)
        {
            RCLCPP_INFO(node_->get_logger(), "SUCCESS!!! enable/disable");
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            RCLCPP_ERROR(node_->get_logger(), "FAILURE!!! enable/disable");
            return BT::NodeStatus::FAILURE;
        }
    }

    return BT::NodeStatus::FAILURE;
}

BT::PortsList BtArmEnable::providedPorts() { return {BT::InputPort<bool>("enable")}; }
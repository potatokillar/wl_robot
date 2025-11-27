/*
 * @Author: 唐文浩
 * @Date: 2025-01-21
 * @LastEditors: 唐文浩
 * @LastEditTime: 2025-02-08
 * @Description:
 *
 * Copyright (c) 2022-2025 by 西湖大学智能产业研究院, All Rights Reserved.
 */

#include "bt_remote_ctrl.hpp"
BtRemoteCtrl::BtRemoteCtrl(const std::string &name, const BT::NodeConfig &conf, std::shared_ptr<rclcpp::Node> node)
    : BT::ThreadedAction(name, conf), node_(node)
{
    cli_trigger_ = node_->create_client<std_srvs::srv::Trigger>("/remote_ctrl/bt_trigger");
    if (!cli_trigger_->wait_for_service(std::chrono::seconds(5)))
    {
        RCLCPP_WARN(node_->get_logger(), "remote_ctrl service not available, will return FAILURE when called");
        service_available_ = false;
    }
    else
    {
        RCLCPP_INFO(node_->get_logger(), "remote_ctrl service available");
        service_available_ = true;
    }
}

BT::NodeStatus BtRemoteCtrl::tick()
{
    // 检查服务是否可用
    if (!service_available_ || !cli_trigger_->service_is_ready())
    {
        RCLCPP_DEBUG(node_->get_logger(), "remote_ctrl service not ready, returning FAILURE");
        return BT::NodeStatus::FAILURE;
    }

    auto ret_future = cli_trigger_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());

    // 超时3秒（与 remote_ctrl 的检测周期对齐）
    auto ret_future_status = ret_future.wait_for(std::chrono::seconds(3));
    if (ret_future_status == std::future_status::ready)
    {
        if (ret_future.get()->success == true)
        {
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            return BT::NodeStatus::FAILURE;
        }
    }
    else if (ret_future_status == std::future_status::timeout)
    {
        RCLCPP_WARN(node_->get_logger(), "remote_ctrl service call timeout (3s), assuming no control");
    }
    return BT::NodeStatus::FAILURE;
}

BT::PortsList BtRemoteCtrl::providedPorts() { return {}; }
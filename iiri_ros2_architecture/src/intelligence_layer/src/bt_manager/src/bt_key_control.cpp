/*
 * @Author: 唐文浩
 * @Date: 2025-01-08
 * @LastEditors: 唐文浩
 * @LastEditTime: 2025-02-08
 * @Description:
 *
 * Copyright (c) 2022-2025 by 西湖大学智能产业研究院, All Rights Reserved.
 */
#include "bt_key_control.hpp"

BtKeyCtrl::BtKeyCtrl(const std::string &name, const BT::NodeConfig &conf, std::shared_ptr<rclcpp::Node> node) : BT::ThreadedAction(name, conf), node_(node)
{
    cli_trigger_ = node_->create_client<std_srvs::srv::Trigger>("/key_control/bt_trigger");
    // 等待服务端准备好
    if (!cli_trigger_->wait_for_service(std::chrono::seconds(5)))
    {
        RCLCPP_FATAL(node_->get_logger(), "service not available, shutdown");
        rclcpp::shutdown();
    }
}

BT::NodeStatus BtKeyCtrl::tick()
{
    auto ret_future = cli_trigger_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());

    // 超时30秒
    auto ret_future_status = ret_future.wait_for(std::chrono::seconds(30));
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
        RCLCPP_ERROR(node_->get_logger(), "service call timeout");
    }
    return BT::NodeStatus::FAILURE;
}

BT::PortsList BtKeyCtrl::providedPorts() { return {BT::InputPort<std::string>("port_name", "default_value", "description")}; }
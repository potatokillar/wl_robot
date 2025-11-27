/*
 * @Author: 唐文浩
 * @Date: 2025-04-21
 * @LastEditors: 唐文浩
 * @LastEditTime: 2025-06-18
 * @Description:
 *
 * Copyright (c) 2022-2025 by 西湖大学智能产业研究院, All Rights Reserved.
 */
#include "global.hpp"
#include "rclcpp/rclcpp.hpp"
#include "xiaozhi_node.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<XiaoZhiNode>();
    g_node_ptr = node;

    // 设置日志级别，从yaml文件获取
    int log_level_from_config = g_node_ptr->declare_parameter<int>("log_level", 3);
    if (log_level_from_config == 2)
    {
        g_node_ptr->get_logger().set_level(rclcpp::Logger::Level::Debug);
    }
    else if (log_level_from_config == 3)
    {
        g_node_ptr->get_logger().set_level(rclcpp::Logger::Level::Info);
    }

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
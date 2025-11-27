/*
 * @Author: 唐文浩
 * @Date: 2025-01-17
 * @LastEditors: 唐文浩
 * @LastEditTime: 2025-02-25
 * @Description:
 *
 * Copyright (c) 2022-2025 by 西湖大学智能产业研究院, All Rights Reserved.
 */

#include "camera.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char *argv[])
{
    // 初始化 ROS 2
    rclcpp::init(argc, argv);

    // 创建节点并保持运行
    auto node = std::make_shared<Camera>();
    rclcpp::spin(node);

    // 关闭 ROS 2
    rclcpp::shutdown();
    return 0;
}
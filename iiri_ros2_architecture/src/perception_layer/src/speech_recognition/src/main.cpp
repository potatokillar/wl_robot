/*
 * @Author: 唐文浩
 * @Date: 2024-10-08
 * @LastEditors: 唐文浩
 * @LastEditTime: 2025-04-21
 * @Description: 鱼香ros2的demo
 *
 * Copyright (c) 2022-2024 by 西湖大学智能产业研究院, All Rights Reserved.
 */
#include <limits.h> // PATH_MAX
#include <unistd.h> // getcwd

#include "AliyunApi.hpp"
#include "CommandExtract.hpp"
#include "Global.hpp"
#include "Microphone.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char *argv[])
{
    char cwd[PATH_MAX];
    if (getcwd(cwd, sizeof(cwd)) != NULL)
    {
        std::cout << "Current working directory: " << cwd << std::endl;
    }
    else
    {
        perror("getcwd() error");
        return 1;
    }

    signal(SIGINT, signal_handler_int);
    signal(SIGQUIT, signal_handler_quit);

    rclcpp::init(argc, argv);
    auto node_ptr = std::make_shared<speech_recognition>();
    g_node_ptr = node_ptr;

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

    // 从yaml文件获取akId和akSecret
    g_akId = g_node_ptr->declare_parameter<std::string>("ak_id", "");
    g_akSecret = g_node_ptr->declare_parameter<std::string>("ak_secret", "");
    g_appkey = g_node_ptr->declare_parameter<std::string>("appkey", "");

    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // 创建子日志器
    auto microphone_logger = g_node_ptr->get_logger().get_child("Microphone");
    auto aliyun_api_logger = g_node_ptr->get_logger().get_child("AliyunApiClient");
    auto cmd_extract_logger = g_node_ptr->get_logger().get_child("CmdExtract");

    // std::thread microphone_thread = std::thread(receiveMicData, microphone_logger);
    // std::thread asr_thread = std::thread(AliyunApiClient, aliyun_api_logger);
    std::thread command_thread = std::thread([&]()
                                             {
        CmdExtract cmd_extract_obj;
        cmd_extract_obj.Work(cmd_extract_logger); });

    rclcpp::executors::MultiThreadedExecutor executor; // 单线程执行器
    executor.add_node(node_ptr);
    executor.spin();

    // microphone_thread.join();
    // asr_thread.join();
    command_thread.join();

    rclcpp::shutdown();
    return 0;
}
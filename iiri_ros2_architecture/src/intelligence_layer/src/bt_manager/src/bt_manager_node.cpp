/*
 * @Author: 唐文浩
 * @Date: 2025-01-08
 * @LastEditors: 唐文浩
 * @LastEditTime: 2025-02-06
 * @Description:
 *
 * Copyright (c) 2022-2025 by 西湖大学智能产业研究院, All Rights Reserved.
 */
#include <behaviortree_cpp/loggers/abstract_logger.h>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>
#include <behaviortree_cpp/loggers/bt_observer.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>

#include <cstdio>
#include <iostream>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "bt_arm_task.hpp"
#include "bt_delay.hpp"
#include "bt_key_control.hpp"
#include "bt_remote_ctrl.hpp"
#include "bt_set_gripper.hpp"
#include "bt_smart_follow.hpp"
#include "bt_speech_recognition.hpp"
#include "interface/srv/exec_tree.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_base/timer_tools.hpp"

using namespace std;
using namespace BT;

std::atomic<bool> g_shutdown_requested(false);

/**
 * @description: 用于ctrl+c后能正确退出
 * @param signal
 * @return {}
 */
void signalHandler(int signal)
{
    if (signal == SIGINT)
    {
        g_shutdown_requested.store(true);
    }
}

struct TreeNodeStatus
{
    double time_;
    std::string node_name_;
    std::string prev_status_;
    std::string status_;
};

// 树观察者，用于监控节点状态变化
class TreeFeedbackLog : public BT::StatusChangeLogger
{
public:
    TreeFeedbackLog(const BT::Tree &tree) : StatusChangeLogger(tree.rootNode()) {}
    void callback(BT::Duration timestamp, const TreeNode &node, NodeStatus prev_status, NodeStatus status) override
    {
        using namespace std::chrono;
        double since_epoch = duration<double>(timestamp).count();
        status_ = {since_epoch, node.name(), toStr(prev_status), toStr(status)};
        new_change_ = true;
    }

    void flush() override { std::cout << std::flush; }

    std::optional<TreeNodeStatus> GetStatus()
    {
        if (new_change_)
        {
            new_change_ = false;
            return status_;
        }
        return std::nullopt;
    }

private:
    TreeNodeStatus status_;
    bool new_change_ = false;
};

/**
 * @description: 默认树管理器 - 负责在服务准备好后启动默认树
 * @author: 唐文浩
 * @date: 2025-10-14
 */
class DefaultTreeManager
{
public:
    DefaultTreeManager(std::shared_ptr<rclcpp::Node> node)
        : node_(node), service_called_(false)
    {
        // 声明并获取默认树参数
        node_->declare_parameter<std::string>("default_tree", "default");
        tree_name_ = node_->get_parameter("default_tree").as_string();
        RCLCPP_INFO(node_->get_logger(), "Default tree configured: %s", tree_name_.c_str());

        // 创建服务客户端
        client_ = node_->create_client<interface::srv::ExecTree>("/bt_manager/exec_tree");

        // 创建定时器，每秒检查一次服务是否可用
        timer_ = node_->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&DefaultTreeManager::checkAndStartDefaultTree, this)
        );
    }

private:
    void checkAndStartDefaultTree()
    {
        // 如果已经调用过服务，直接返回
        if (service_called_)
        {
            return;
        }

        // 检查服务是否可用
        if (!client_->wait_for_service(std::chrono::seconds(0)))
        {
            RCLCPP_DEBUG(node_->get_logger(), "Waiting for bt_manager service to be available...");
            return;
        }

        // 服务可用，发送请求
        RCLCPP_INFO(node_->get_logger(), "Service available, starting default tree: %s", tree_name_.c_str());

        auto request = std::make_shared<interface::srv::ExecTree::Request>();
        request->tree = tree_name_;

        // 异步发送请求
        auto future = client_->async_send_request(request,
            [this](rclcpp::Client<interface::srv::ExecTree>::SharedFuture future)
            {
                try
                {
                    auto response = future.get();
                    if (response->success)
                    {
                        RCLCPP_INFO(node_->get_logger(), "Default tree started successfully");
                    }
                    else
                    {
                        RCLCPP_ERROR(node_->get_logger(), "Failed to start default tree: %s",
                                    response->message.c_str());
                    }
                }
                catch (const std::exception &e)
                {
                    RCLCPP_ERROR(node_->get_logger(), "Service call failed: %s", e.what());
                }
            }
        );

        // 标记已调用，取消定时器
        service_called_ = true;
        timer_->cancel();
        RCLCPP_INFO(node_->get_logger(), "Default tree request sent, timer cancelled");
    }

private:
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Client<interface::srv::ExecTree>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string tree_name_;
    bool service_called_;
};

// 树执行器，可通过服务加载新的树
class TreeExecutor
{
public:
    /**
     * @description:
     * @param exec_node 树执行器的ros节点
     * @param leaf_node 叶子节点的ros节点
     * @return {}
     */
    TreeExecutor(std::shared_ptr<rclcpp::Node> exec_node, std::shared_ptr<rclcpp::Node> leaf_node) : ros_node_(exec_node)
    {
        // 注册所有的节点类型，目前暂时由本包统一管理
        factory_.registerNodeType<BtKeyCtrl>("BtKeyCtrl", leaf_node);
        factory_.registerNodeType<BtSmartFollow>("BtSmartFollow", leaf_node);
        factory_.registerNodeType<BtArmMoveJ>("BtArmMoveJ", leaf_node);
        factory_.registerNodeType<BtArmMoveL>("BtArmMoveL", leaf_node);
        factory_.registerNodeType<BtArmEnable>("BtArmEnable", leaf_node);
        factory_.registerNodeType<BtSpeechRecognition>("BtSpeechRecognition", leaf_node);
        factory_.registerNodeType<BtSetGripper>("BtSetGripper", leaf_node);
        factory_.registerNodeType<BtDelay>("BtDelay", leaf_node);
        factory_.registerNodeType<BtRemoteCtrl>("BtRemoteCtrl", leaf_node);

        srv_exec_tree_ = ros_node_->create_service<interface::srv::ExecTree>(
            "/bt_manager/exec_tree",
            [this](const std::shared_ptr<interface::srv::ExecTree::Request> request, std::shared_ptr<interface::srv::ExecTree::Response> response)
            {
                this->RxExecTree(request, response);
            });
    }

private:
    void RxExecTree(const std::shared_ptr<interface::srv::ExecTree::Request> request, std::shared_ptr<interface::srv::ExecTree::Response> response)
    {
        RCLCPP_INFO(ros_node_->get_logger(), "enter exectree");

        try
        {
            std::string config_file_path = package_share_dir_ + "/config/tree/" + request->tree + ".xml";
            RCLCPP_INFO(ros_node_->get_logger(), "Received goal request: %s, path:%s", request->tree.c_str(), config_file_path.c_str());
            auto tree = factory_.createTreeFromFile(config_file_path);
            auto groot = std::make_shared<BT::Groot2Publisher>(tree, 1667);
            // StdCoutLogger logger(tree);
            auto status = BT::NodeStatus::RUNNING;
            // 会阻塞在这里，根据树的编辑，可能会死循环
            while (rclcpp::ok() && status == BT::NodeStatus::RUNNING && !g_shutdown_requested.load())
            {
                status = tree.tickExactlyOnce();
                RCLCPP_DEBUG(ros_node_->get_logger(), "Status: %s", toStr(status).c_str());
                std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 非精确的延时
            }
            response->success = true;
        }
        catch (std::exception &e)
        {
            RCLCPP_ERROR(ros_node_->get_logger(), "Failed to load tree: %s, %s", request->tree.c_str(), e.what());
            response->success = false;
            response->message = e.what();
        }
    }

private:
    std::shared_ptr<rclcpp::Node> ros_node_;
    BehaviorTreeFactory factory_;
    // 获取包的共享目录路径
    std::string package_share_dir_ = ament_index_cpp::get_package_share_directory("bt_manager");
    rclcpp::Service<interface::srv::ExecTree>::SharedPtr srv_exec_tree_;
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // 创建三个节点：执行器节点、叶子节点、默认树管理节点
    auto exec_node = std::make_shared<rclcpp::Node>("bt_manager");
    auto leaf_node = std::make_shared<rclcpp::Node>("bt_leaf");
    auto default_node = std::make_shared<rclcpp::Node>("bt_default_tree");

    RCLCPP_INFO(exec_node->get_logger(), "Hello bt_node!");

    // 注册信号处理函数
    std::signal(SIGINT, signalHandler);

    // 创建树执行器，负责提供exec_tree服务
    TreeExecutor executor(exec_node, leaf_node);

    // 创建默认树管理器，负责在服务准备好后启动默认树
    DefaultTreeManager default_manager(default_node);

    // 将所有三个节点添加到多线程执行器
    rclcpp::executors::MultiThreadedExecutor multi_executor;
    multi_executor.add_node(exec_node);
    multi_executor.add_node(leaf_node);
    multi_executor.add_node(default_node);

    RCLCPP_INFO(exec_node->get_logger(), "All nodes initialized, starting executor spin");
    multi_executor.spin();

    rclcpp::shutdown();
    return 0;
}

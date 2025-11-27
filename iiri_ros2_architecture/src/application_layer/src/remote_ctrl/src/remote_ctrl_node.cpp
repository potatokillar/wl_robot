/*
 * @Author: 唐文浩
 * @Date: 2025-01-21
 * @LastEditors: 唐文浩
 * @LastEditTime: 2025-06-10
 * @Description: 远程控制，目前mqtt有关的均在这里，将来提取mqtt出去
 *
 * Copyright (c) 2022-2025 by 西湖大学智能产业研究院, All Rights Reserved.
 */
#include <arpa/inet.h>
#include <ifaddrs.h>
#include <mqtt/client.h>

#include <mutex>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "interface/msg/camera_ptz_move.hpp"
#include "interface/msg/pose.hpp"
#include "interface/msg/run_state.hpp"
#include "interface/srv/my_info.hpp"
#include "robot_base/cppSqlite.hpp"
#include "robot_base/timer_tools.hpp"
#include "robot_base/ws_client.hpp"

using namespace std;
using namespace nlohmann;
class RemoteCtrl : public rclcpp::Node
{
    using Trigger = std_srvs::srv::Trigger;
    using String = std_msgs::msg::String;

public:
    RemoteCtrl() : Node("remote_ctrl_node")
    {
        RCLCPP_INFO(this->get_logger(), "remote_ctrl_node start");
        vector<string> tmp;
        this->declare_parameter<vector<string>>("net_name", tmp);
        srv_trigger_ = create_service<Trigger>(
            "/remote_ctrl/bt_trigger",
            [this](const std::shared_ptr<Trigger::Request> request, std::shared_ptr<Trigger::Response> response)
            { this->on_trigger(request, response); });

        pub_vel_ = create_publisher<geometry_msgs::msg::Twist>("/motion_control/cmd_vel", 10);
        pub_pos_ = create_publisher<interface::msg::Pose>("/motion_control/cmd_pos", 10);
        pub_run_state_ = create_publisher<interface::msg::RunState>("/motion_control/cmd_run_state", 10);
        pub_ptz_ = create_publisher<interface::msg::CameraPTZMove>("/camera_ptz/camera_ptz_move", 10);

        // 只要服务可用，即节点已起动。
        auto cli_ws_ready = this->create_client<std_srvs::srv::Empty>("/dev_server/ready");
        cli_ws_ready->wait_for_service();

        cli_my_info_ = create_client<interface::srv::MyInfo>("/robot_base/my_info");
        cli_my_info_->wait_for_service();
        cli_my_info_->async_send_request(std::make_shared<interface::srv::MyInfo::Request>(),
                                         [this](rclcpp::Client<interface::srv::MyInfo>::SharedFuture future)
                                         {
                                             auto response = future.get();
                                             this->init_connect(response->uuid, response->addr);
                                         });

        RCLCPP_INFO(this->get_logger(), "remote_ctrl_node init complete");
    }

    virtual ~RemoteCtrl() {}

private:
    void on_trigger(const std::shared_ptr<Trigger::Request> request, std::shared_ptr<Trigger::Response> response)
    {
        (void)request;
        // response->success = true;

        if (TimerTools::GetNowTickMs() - last_recv_ > 3000)
        {
            response->success = false;
        }
        else
        {
            response->success = true;
        }
    }

    void init_connect(const std::string &uuid, const std::array<std::string, 3> &srv_ip)
    {
        uuid_ = uuid;
        RCLCPP_INFO(this->get_logger(), "uuid: %s", uuid.c_str());

        vector<string> paths = {"/quadruped/set_velocity", "/quadruped/set_pos", "/quadruped/set_run_state"};
        // 暂时只支持一个todo
        if (!srv_ip[0].empty())
        {
            auto client = std::make_shared<WebSocketClient>(srv_ip[0], uuid, paths);
            client->async_recv([this](const std::string &path, const std::string &data)
                               { this->on_recv(path, data); });
            ws_clients_.push_back(client);
            ws_threads_.emplace_back(std::thread([client]()
                                                 { client->loop(); }));
        }

#if 0
        // 启动多个连接任务
        for (size_t i = 0; i < srv_ip.size(); i++) {
            if (!srv_ip[i].empty()) {
                auto client = std::make_shared<WebSocketClient>(srv_ip[i], uuid, paths);
                client->async_recv([this](const std::string& path, const std::string& data) { this->on_recv(path, data); });
                ws_clients_.push_back(client);
                ws_threads_.emplace_back(std::thread([client]() { client->loop(); }));
            }
        }
#endif
    }

    void on_recv(const std::string &path, const std::string &payload)
    {
        RCLCPP_INFO(this->get_logger(), "[REMOTE_CTRL] WS received: path=%s, payload=%s",
                    path.c_str(), payload.substr(0, 200).c_str());
        if (path == "/quadruped/set_velocity")
        {
            pub_velocity(payload);
        }
        else if (path == "/quadruped/set_run_state")
        {
            pub_run_state(payload);
        }
        else if (path == "/quadruped/set_pose")
        {
            pub_position(payload);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "[REMOTE_CTRL] Unknown path: %s", path.c_str());
        }
    }

    void pub_velocity(const std::string &payload)
    {
        try
        {
            nlohmann::json json = nlohmann::json::parse(payload);
            nlohmann::json json_param = json.at("param");
            geometry_msgs::msg::Twist twist;
            twist.linear.x = std::stod(json_param.at("linear").at(0).get<std::string>());
            twist.linear.y = std::stod(json_param.at("linear").at(1).get<std::string>());
            twist.angular.z = std::stod(json_param.at("angular").at(2).get<std::string>());
            pub_vel_->publish(twist);
            RCLCPP_INFO(this->get_logger(), "[REMOTE_CTRL] ROS2 published velocity: linear=[%.2f, %.2f], angular_z=[%.2f]",
                        twist.linear.x, twist.linear.y, twist.angular.z);
        }
        catch (nlohmann::json::exception &e)
        {
            RCLCPP_WARN(this->get_logger(), "[REMOTE_CTRL] velocity json error: %s", e.what());
        }
    }

    void pub_run_state(const std::string &payload)
    {
        try
        {
            interface::msg::RunState run_state;
            nlohmann::json json = nlohmann::json::parse(payload);
            nlohmann::json json_param = json.at("param");
            if (json_param.at("run_state") == "stand")
            {
                run_state.value = interface::msg::RunState::STAND;
            }
            else if (json_param.at("run_state") == "walk")
            {
                run_state.value = interface::msg::RunState::WALK;
            }
            else if (json_param.at("run_state") == "lie")
            {
                run_state.value = interface::msg::RunState::LIE;
            }
            pub_run_state_->publish(run_state);
            RCLCPP_INFO(this->get_logger(), "[REMOTE_CTRL] ROS2 published run_state=%s (value=%d)",
                        json_param.at("run_state").get<std::string>().c_str(), run_state.value);
        }
        catch (nlohmann::json::exception &e)
        {
            RCLCPP_WARN(this->get_logger(), "run_state json error: %s", e.what());
        }
    }

    void pub_position(const std::string &payload)
    {
        try
        {
            interface::msg::Pose pose;
            nlohmann::json json = nlohmann::json::parse(payload);
            pose.roll = json.at("roll");
            pose.pitch = json.at("pitch");
            pose.yaw = json.at("yaw");
            pub_pos_->publish(pose);
            RCLCPP_INFO(this->get_logger(), "position: %f, %f, %f", pose.roll, pose.pitch, pose.yaw);
        }
        catch (nlohmann::json::exception &e)
        {
            RCLCPP_WARN(this->get_logger(), "position json error: %s", e.what());
        }
    }

    void pub_ptz(const std::string &payload)
    {
        try
        {
            interface::msg::CameraPTZMove ptz;
            nlohmann::json json = nlohmann::json::parse(payload);
            ptz.vx = Json2Double(json.at("param").at("vx"));
            ptz.vy = Json2Double(json.at("param").at("vy"));
            pub_ptz_->publish(ptz);
            RCLCPP_INFO(this->get_logger(), "ptz: %f, %f", ptz.vx, ptz.vy);
        }
        catch (nlohmann::json::exception &e)
        {
            RCLCPP_WARN(this->get_logger(), "ptz json error: %s", e.what());
        }
    }

private:
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_trigger_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_vel_;
    rclcpp::Publisher<interface::msg::Pose>::SharedPtr pub_pos_;
    rclcpp::Publisher<interface::msg::RunState>::SharedPtr pub_run_state_;
    rclcpp::Publisher<interface::msg::CameraPTZMove>::SharedPtr pub_ptz_;

    rclcpp::Client<interface::srv::MyInfo>::SharedPtr cli_my_info_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr cli_ws_ready_;

    std::string uuid_;

    bool mqtt_running_ = false;
    uint64_t last_recv_ = 0; // 上一次mqtt接收数据的时间

    std::vector<std::shared_ptr<WebSocketClient>> ws_clients_;
    std::vector<std::thread> ws_threads_;
    std::mutex mtx_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RemoteCtrl>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

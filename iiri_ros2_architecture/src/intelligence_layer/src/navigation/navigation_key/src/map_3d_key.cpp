/*
 * @Author: 唐文浩
 * @Date: 2024-12-13
 * @LastEditors: 周健伟
 * @LastEditTime: 2025-06-21
 * @Description: 智能跟随数据读取发布节点
 *
 * Copyright (c) 2022-2024 by 西湖大学智能产业研究院, All Rights Reserved.
 */
#include <deque>
#include <rclcpp/rclcpp.hpp>

#include "geometry_msgs/msg/twist.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_srvs/srv/empty.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "interface/msg/gamepad_cmd.hpp"

namespace navigation_key
{

    using namespace std::placeholders;

    class Map3dKey : public rclcpp::Node
    {
    public:
        Map3dKey() : Node("navigation_and_record_key_node")
        {
            last_rb_ = false;
            gamepad_sub_ = this->create_subscription<interface::msg::GamepadCmd>(
                "/motion_control/ret_gamepad_cmd", 10,
                std::bind(&Map3dKey::gamepad_callback, this, _1));
            tts_content_pub_ = this->create_publisher<std_msgs::msg::String>("/tts/content", 10);
            record_horizontal_point_client_ = create_client<std_srvs::srv::Trigger>("record_horizontal_point");
            save_horizontal_map_client_ = create_client<std_srvs::srv::Trigger>("save_horizontal_map");
            into_map_record_time_ = 0;
        }

    private:
        void gamepad_callback(const interface::msg::GamepadCmd::SharedPtr msg)
        {
            if (msg->rb && !last_rb_)
            {
                // 记录水平地图
                ++into_map_record_time_;

                if (into_map_record_time_ % 4 > 0)
                {
                    if (!record_horizontal_point_client_->wait_for_service(std::chrono::seconds(1)))
                    {
                        if (!rclcpp::ok())
                        {
                            RCLCPP_ERROR(this->get_logger(), "unknown error! rclcpp is shutdown!");
                            return;
                        }
                        RCLCPP_WARN(this->get_logger(), "record_horizontal_point service is not online");
                    }
                    record_horizontal_point_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());

                    // 语音播报
                    auto str_to_tts = std_msgs::msg::String();
                    str_to_tts.data = "记录" + std::to_string(into_map_record_time_ % 4) + "个水平点";
                    tts_content_pub_->publish(str_to_tts);
                }
                else
                {
                    into_map_record_time_ = 0;
                    if (!save_horizontal_map_client_->wait_for_service(std::chrono::seconds(1)))
                    {
                        if (!rclcpp::ok())
                        {
                            RCLCPP_ERROR(this->get_logger(), "unknown error! rclcpp is shutdown!");
                            return;
                        }
                        RCLCPP_WARN(this->get_logger(), "save_horizontal_map service is not online");
                    }
                    save_horizontal_map_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
                    // 语音播报
                    auto str_to_tts = std_msgs::msg::String();
                    str_to_tts.data = "保存对齐水平地图";
                    tts_content_pub_->publish(str_to_tts);
                }
            }
        }

    private:
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr tts_content_pub_;
        rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr record_horizontal_point_client_; // 记录水平点触发客户端
        rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr save_horizontal_map_client_;     // 记录地图客户端
        int into_map_record_time_;                                                         // 进入地图保存按键分支次数，对其%4，123次都为记录水平点，0为保存地图文件到resource文件中。

        bool last_rb_ = false;

        rclcpp::Subscription<interface::msg::GamepadCmd>::SharedPtr gamepad_sub_;
    };

}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<navigation_key::Map3dKey>();
    RCLCPP_INFO(node->get_logger(), "Hello map key node");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
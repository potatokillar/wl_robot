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
#include "std_msgs/msg/empty.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "interface/msg/gamepad_cmd.hpp"

namespace navigation_key
{

    using namespace std::placeholders;

    class RecordTrajKey : public rclcpp::Node
    {
    public:
        RecordTrajKey() : Node("record_traj_key_node")
        {

            gamepad_sub_ = this->create_subscription<interface::msg::GamepadCmd>(
                "/motion_control/ret_gamepad_cmd", 10,
                std::bind(&RecordTrajKey::gamepad_callback, this, _1));
            speaker_pub_ = this->create_publisher<std_msgs::msg::String>("/speaker/wav_file", 10);
            tts_content_pub_ = this->create_publisher<std_msgs::msg::String>("/tts/content", 10);
            start_record_traj_client_ = create_client<std_srvs::srv::Trigger>("start_record_traj");
            stop_record_traj_client_ = create_client<std_srvs::srv::Trigger>("stop_record_traj");
            save_traj_client_ = create_client<std_srvs::srv::Trigger>("save_traj");
            relocalize_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            relocalize_client_ = this->create_client<std_srvs::srv::Empty>("relocalize", rmw_qos_profile_services_default, relocalize_callback_group_);
            tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
            press_rb_count_ = 0;
            last_rb_ = last_y_ = false;
            last_rt_ = 0.0;
        }

    private:
        void gamepad_callback(const interface::msg::GamepadCmd::SharedPtr msg)
        {
            if (msg->rb && !last_rb_)
            {
                // 记录轨迹
                ++press_rb_count_;

                if (press_rb_count_ % 2 == 1)
                {
                    if (!start_record_traj_client_->wait_for_service(std::chrono::seconds(1)))
                    {
                        if (!rclcpp::ok())
                        {
                            RCLCPP_ERROR(this->get_logger(), "unknown error! rclcpp is shutdown!");
                            return;
                        }
                        RCLCPP_WARN(this->get_logger(), "start_record_traj service is not online");
                        return;
                    }
                    start_record_traj_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());

                    // 语音播报
                    auto str_to_speak = std_msgs::msg::String();
                    str_to_speak.data = "start_record_traj";
                    speaker_pub_->publish(str_to_speak);
                }
                else
                {
                    press_rb_count_ = 0;
                    if (!stop_record_traj_client_->wait_for_service(std::chrono::seconds(1)))
                    {
                        if (!rclcpp::ok())
                        {
                            RCLCPP_ERROR(this->get_logger(), "unknown error! rclcpp is shutdown!");
                            return;
                        }
                        RCLCPP_WARN(this->get_logger(), "stop_record_traj service is not online");
                        return;
                    }
                    stop_record_traj_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
                    // 语音播报
                    auto str_to_speak = std_msgs::msg::String();
                    str_to_speak.data = "stop_record_traj";
                    speaker_pub_->publish(str_to_speak);
                }
            }
            else if (msg->rt && !last_rt_)
            {
                if (!save_traj_client_->wait_for_service(std::chrono::seconds(1)))
                {
                    if (!rclcpp::ok())
                    {
                        RCLCPP_ERROR(this->get_logger(), "unknown error! rclcpp is shutdown!");
                        return;
                    }
                    RCLCPP_WARN(this->get_logger(), "save_traj service is not online");
                    return;
                }
                save_traj_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
                // 语音播报
                auto str_to_speak = std_msgs::msg::String();
                str_to_speak.data = "save_traj";
                speaker_pub_->publish(str_to_speak);
            }
            else if (msg->y && !last_y_)
            {
                if (!relocalize_client_->wait_for_service(std::chrono::seconds(2)))
                {
                    RCLCPP_WARN(this->get_logger(), "relocalize service is not online !");
                    return;
                }
                auto request = std::make_shared<std_srvs::srv::Empty::Request>();
                auto relocaliza_future = relocalize_client_->async_send_request(request);

                if (relocaliza_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready)
                {
                    RCLCPP_WARN(this->get_logger(), "wait for service response timeout !");
                    return;
                };
                RCLCPP_INFO(this->get_logger(), "Relocalize success !");

                geometry_msgs::msg::TransformStamped transform;
                rclcpp::sleep_for(std::chrono::milliseconds(200));
                transform = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);

                std_msgs::msg::String relocalize_msg;
                relocalize_msg.data = "当前位置: (" +
                                      std::to_string(float(transform.transform.translation.x)).substr(0, std::to_string(float(transform.transform.translation.x)).find('.') + 3) + ", " +
                                      std::to_string(float(transform.transform.translation.y)).substr(0, std::to_string(float(transform.transform.translation.y)).find('.') + 3) + ", " +
                                      std::to_string(float(transform.transform.translation.z)).substr(0, std::to_string(float(transform.transform.translation.z)).find('.') + 3) + ")";
                tts_content_pub_->publish(relocalize_msg);
                RCLCPP_WARN(this->get_logger(), "Current position: (%f, %f, %f)",
                            transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z);
                RCLCPP_WARN(this->get_logger(), "Current orientation: (%f, %f, %f, %f)",
                            transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w);
            }
            last_rb_ = msg->rb;
            last_rt_ = msg->rt;
            last_y_ = msg->y;
        }

    private:
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr speaker_pub_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr tts_content_pub_;
        rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_record_traj_client_; // 开始记录轨迹客户端
        rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_record_traj_client_;  // 停止记录轨迹客户端
        rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr save_traj_client_;         // 保存轨迹客户端
        rclcpp::Client<std_srvs::srv::Empty>::SharedPtr relocalize_client_;          // 重定位客户端
        rclcpp::CallbackGroup::SharedPtr relocalize_callback_group_;                 // 接收重定位服务回调函数组
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        int press_rb_count_;

        bool last_rb_, last_y_;
        double last_rt_;

        rclcpp::Subscription<interface::msg::GamepadCmd>::SharedPtr gamepad_sub_;
    };

}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor exec;
    auto node = std::make_shared<navigation_key::RecordTrajKey>();
    RCLCPP_INFO(node->get_logger(), "Hello record_traj_key_node");
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
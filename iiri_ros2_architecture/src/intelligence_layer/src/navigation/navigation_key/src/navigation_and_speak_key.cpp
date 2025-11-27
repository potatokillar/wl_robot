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

    class NavigationAndSpeakKey : public rclcpp::Node
    {
    public:
        NavigationAndSpeakKey() : Node("navigation_and_speak_key_node")
        {
            this->declare_parameter<std::vector<std::string>>("trajectories", {"traj-2025-06-18-19:19:05.csv"});
            this->declare_parameter<std::vector<std::string>>("audios", {"openning_speech"});

            trajectories_ = this->get_parameter("trajectories").as_string_array();
            audios_ = this->get_parameter("audios").as_string_array();

            gamepad_sub_ = this->create_subscription<interface::msg::GamepadCmd>(
                "/motion_control/ret_gamepad_cmd", 10,
                std::bind(&NavigationAndSpeakKey::gamepad_callback, this, _1));
            speaker_pub_ = this->create_publisher<std_msgs::msg::String>("/speaker/wav_file", 10);
            tts_content_pub_ = this->create_publisher<std_msgs::msg::String>("/tts/content", 10);
            path_tracker_send_goal_pub_ = this->create_publisher<std_msgs::msg::String>("send_track_path_goal", 10);
            path_tracker_cancel_pub_ = this->create_publisher<std_msgs::msg::Empty>("cancel_track_path_goal", 10);
            relocalize_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            relocalize_client_ = this->create_client<std_srvs::srv::Empty>("relocalize", rmw_qos_profile_services_default, relocalize_callback_group_);
            tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
            press_rb_count_ = 0;
            press_lt_count_ = 0;
            press_rt_count_ = 0;
            last_rb_ = last_lb_ = last_y_ = false;
            last_rt_ = last_lt_ = 0.0;
        }

    private:
        void gamepad_callback(const interface::msg::GamepadCmd::SharedPtr msg)
        {
            if (msg->rb && !last_rb_)
            {

                press_rb_count_++;
                if (press_rb_count_ % 2)
                {
                    std_msgs::msg::String traj_forward;
                    traj_forward.data = trajectories_[press_rt_count_ % trajectories_.size()];
                    path_tracker_send_goal_pub_->publish(traj_forward);
                    press_rt_count_ += 1;
                    // 语音播报
                    std_msgs::msg::String str_to_speaker;
                    str_to_speaker.data = "route_navigation";
                    speaker_pub_->publish(str_to_speaker);
                }
                else
                {
                    path_tracker_cancel_pub_->publish(std_msgs::msg::Empty());
                }
            }
            else if (msg->lb && !last_lb_)
            {
                std_msgs::msg::String current_audio;
                current_audio.data = this->audios_[press_lt_count_ % audios_.size()];
                speaker_pub_->publish(current_audio);
                press_lt_count_ += 1;
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
            else if (msg->rt > 0.9 && last_rt_ < 0.9)
            {
                // 语音播报
                press_rt_count_ += 1;
                std_msgs::msg::String str_to_speak;
                str_to_speak.data = "select_route_" + std::to_string(press_rt_count_ % trajectories_.size());
                RCLCPP_WARN(this->get_logger(), "%s", str_to_speak.data.c_str());
                speaker_pub_->publish(str_to_speak);
            }
            else if (msg->lt > 0.9 && last_lt_ < 0.9)
            {
                press_lt_count_ += 1;
                std_msgs::msg::String str_to_speak;
                str_to_speak.data = "select_voice_" + std::to_string(press_lt_count_ % audios_.size());
                RCLCPP_WARN(this->get_logger(), "%s", str_to_speak.data.c_str());
                speaker_pub_->publish(str_to_speak);
            }
            last_rb_ = msg->rb;
            last_lb_ = msg->lb;
            last_rt_ = msg->rt;
            last_lt_ = msg->lt;
            last_y_ = msg->y;
        }

    private:
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr speaker_pub_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr tts_content_pub_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr path_tracker_send_goal_pub_; // 路径跟踪服务发起客户端
        rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr path_tracker_cancel_pub_;     // 路径跟踪服务取消客户端
        rclcpp::Client<std_srvs::srv::Empty>::SharedPtr relocalize_client_;              // 重定位客户端
        rclcpp::CallbackGroup::SharedPtr relocalize_callback_group_;                     // 接收重定位服务回调函数组
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        std::vector<std::string> trajectories_;
        std::vector<std::string> audios_;
        int press_rb_count_;
        int press_rt_count_;
        int press_lt_count_;

        bool last_rb_, last_lb_, last_y_;
        double last_rt_, last_lt_;

        rclcpp::Subscription<interface::msg::GamepadCmd>::SharedPtr gamepad_sub_;
    };

}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor exec;
    auto node = std::make_shared<navigation_key::NavigationAndSpeakKey>();
    RCLCPP_INFO(node->get_logger(), "Hello navigation_and_speak_key_node");
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
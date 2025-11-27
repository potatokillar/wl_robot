/*
 * @Author: 唐文浩
 * @Date: 2024-11-06
 * @LastEditors: 唐文浩
 * @LastEditTime: 2025-06-26
 * @Description: 四足按键部分
 *
 * Copyright (c) 2022-2024 by 西湖大学智能产业研究院, All Rights Reserved.
 */
#include "gamepad.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "interface/msg/battery_info.hpp"
#include "interface/msg/pose.hpp"
#include "interface/msg/quad_state.hpp"
#include "interface/msg/robot_state.hpp"
#include "interface/msg/run_state.hpp"
#include "interface/msg/smart_follow.hpp"
#include "interface/msg/walk_mode.hpp"
#include "interface/srv/dance.hpp"
#include "interface/srv/device_info.hpp"
#include "interface/srv/enable_speech_recog.hpp"
#include "keyBase.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

class KeyQuadruped : public rclcpp::Node
{
public:
    KeyQuadruped() : Node("key_qr")
    {
        this->declare_parameter<std::string>("js", "/dev/input/js0");
        js_ = this->get_parameter("js").as_string();
        RCLCPP_INFO(this->get_logger(), "js: %s", js_.c_str());
        // 10ms扫描一次
        // timerRead_ = this->create_wall_timer(std::chrono::milliseconds(5), [this]() { this->GamepadScan(); });
        // timerCheck_ = this->create_wall_timer(std::chrono::seconds(2), [this]() { this->Check(); });

        sub_vel_ = create_subscription<geometry_msgs::msg::Twist>("/motion_control/ret_vel", 10, [this](const geometry_msgs::msg::Twist::SharedPtr msg)
                                                                  { this->RxVel(msg); });
        sub_pos_ =
            create_subscription<interface::msg::Pose>("/motion_control/ret_pos", 10, [this](const interface::msg::Pose::SharedPtr msg)
                                                      { this->RxPos(msg); });
        sub_qr_state_ = create_subscription<interface::msg::QuadState>("/motion_control/ret_quad_state",
                                                                       10,
                                                                       [this](const interface::msg::QuadState::SharedPtr msg)
                                                                       { this->RxQuadState(msg); });
        sub_run_state_ = create_subscription<interface::msg::RunState>("/motion_control/ret_run_state",
                                                                       10,
                                                                       [this](const interface::msg::RunState::SharedPtr msg)
                                                                       { this->RxRunState(msg); });
        sub_walk_mode_ = create_subscription<interface::msg::WalkMode>("/motion_control/ret_walk_mode",
                                                                       10,
                                                                       [this](const interface::msg::WalkMode::SharedPtr msg)
                                                                       { this->RxWalkMode(msg); });
        sub_robot_state_ =
            create_subscription<interface::msg::RobotState>("/motion_control/ret_robot_state", 10, [this](const interface::msg::RobotState::SharedPtr msg)
                                                            { this->RxRobotState(msg); });
        sub_battery_ = create_subscription<interface::msg::BatteryInfo>("/motion_control/ret_battery_info",
                                                                        10,
                                                                        [this](const interface::msg::BatteryInfo::SharedPtr msg)
                                                                        { this->RxBattery(msg); });
        sub_smart_follow_ =
            create_subscription<interface::msg::SmartFollow>("/motion_control/sensor_smart_follow",
                                                             10,
                                                             [this](const interface::msg::SmartFollow::SharedPtr msg)
                                                             { this->RxSmartFollow(msg); });
        _dance_cli_ptr = this->create_client<interface::srv::Dance>("/dance", rmw_qos_profile_services_default, client_cb_group_);
        pub_vel_ = create_publisher<geometry_msgs::msg::Twist>("/motion_control/cmd_vel", 10);
        pub_pos_ = create_publisher<interface::msg::Pose>("/motion_control/cmd_pos", 10);
        pub_qr_state_ = create_publisher<interface::msg::QuadState>("/motion_control/cmd_quad_state", 10);
        pub_run_state_ = create_publisher<interface::msg::RunState>("/motion_control/cmd_run_state", 10);
        pub_walk_mode_ = create_publisher<interface::msg::WalkMode>("/motion_control/cmd_walk_mode", 10);
        _tts_content_pub_ptr = create_publisher<std_msgs::msg::String>("/tts/content", 10);
        pub_emerg_stop_ = create_publisher<std_msgs::msg::Bool>("/motion_control/cmd_emerg_stop", 10);

        cli_device_info_ = create_client<interface::srv::DeviceInfo>("/device_info");

        pub_open_smart_follow_ = create_publisher<std_msgs::msg::Bool>("/smart_follow/open", 10);

        srv_trigger_ = create_service<std_srvs::srv::Trigger>(
            "/key_control/bt_trigger",
            [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response)
            {
                this->RxTrigger(request, response);
            },
            rmw_qos_profile_services_default,
            service_cb_group_);
        // cli_enable_speech_recog_ = create_client<interface::srv::EnableSpeechRecog>("/EnableSpeechRecog");

        // 初始化回调组
        client_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        service_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    }
    /**
     * @description: 扫描按键
     * @return {}
     */
    bool GamepadScan()
    {
        auto jsData = GamepadRead(js_);
        if (jsData.has_value())
        {
            key_.SaveGamepadBtn(jsData.value());
        }
        else
        {
            is_key_exist_ = false;
            return false;
            // 无按键输入，暂不做处理
            // RCLCPP_INFO(this->get_logger(), "gamepad no key press");
        }

        if (key_.IsShortRelease("start"))
        {
            std_msgs::msg::Bool local_emerg_stop;
            local_emerg_stop.data = false;
            pub_emerg_stop_->publish(local_emerg_stop);
            RCLCPP_INFO(this->get_logger(), "START short press");
        }
        else if (key_.IsShortRelease("back"))
        {
            std_msgs::msg::Bool local_emerg_stop;
            local_emerg_stop.data = true;
            pub_emerg_stop_->publish(local_emerg_stop);
            RCLCPP_INFO(this->get_logger(), "BACK short press");
        }

        //  读取数据
        if (key_.IsShortRelease("a"))
        {
            RCLCPP_INFO(this->get_logger(), "btnA press");
            interface::msg::RunState sta;
            sta.value = interface::msg::RunState::STAND;
            pub_run_state_->publish(sta);
        }
        if (key_.IsShortRelease("b"))
        {
            RCLCPP_INFO(this->get_logger(), "btnB press");
            interface::msg::RunState sta;
            sta.value = interface::msg::RunState::WALK;
            pub_run_state_->publish(sta);
        }
        if (key_.IsShortRelease("x"))
        {
            RCLCPP_INFO(this->get_logger(), "btnX press");
            interface::msg::RunState sta;
            sta.value = interface::msg::RunState::LIE;
            pub_run_state_->publish(sta);
        }

        if (is_key_exist_ == false)
        {
            is_smart_follow_ = false;
            std_msgs::msg::Bool msg;
            msg.data = is_smart_follow_;
            pub_open_smart_follow_->publish(msg);
        }

        if (key_.IsShortRelease("y"))
        {
            is_smart_follow_ = !is_smart_follow_;
            RCLCPP_INFO(this->get_logger(), "smart_follow_open %d", is_smart_follow_);

            std_msgs::msg::Bool msg;
            msg.data = is_smart_follow_;
            pub_open_smart_follow_->publish(msg);
            // 语音播报
            // auto str_to_tts = std_msgs::msg::String();
            // str_to_tts.data = "开启智能跟随";
            // _tts_content_pub_ptr->publish(str_to_tts);
        }

        {
            // 运动
            if (run_sta_.value == interface::msg::RunState::STAND)
            {
                interface::msg::Pose pos;
                pos.roll = 0;
                pos.pitch = key_.GetStick().at("ly") * 0.3;
                pos.yaw = key_.GetStick().at("rx") * 0.3;
                pub_pos_->publish(pos);
            }
            else if (run_sta_.value == interface::msg::RunState::WALK)
            {
                geometry_msgs::msg::Twist vel;
                vel.linear.x = key_.GetStick().at("ly") * -1.0;
                vel.linear.y = key_.GetStick().at("lx") * -0.6;
                vel.angular.z = key_.GetStick().at("rx") * -1.5;
                pub_vel_->publish(vel);
            }
        }
        is_key_exist_ = true;
        return true;
    }

private:
    void RxVel(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // RCLCPP_INFO(this->get_logger(), "Received message: linear.x=%f, angular.z=%f", msg->linear.x,
        // msg->angular.z);
        vel_ = *msg;
    }

    void RxPos(const interface::msg::Pose::SharedPtr msg) { pos_ = *msg; }

    void RxQuadState(const interface::msg::QuadState::SharedPtr msg)
    {
        // RCLCPP_INFO(this->get_logger(), "Received message: state=%d", msg->height);
        qr_sta_ = *msg;
    }

    void RxRunState(const interface::msg::RunState::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Rx:ret_run_state=%d", msg->value);
        run_sta_ = *msg;
    }

    void RxWalkMode(const interface::msg::WalkMode::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Rx:ret_walk_mode=%d", msg->value);
        walk_mode_ = *msg;
    }

    void RxRobotState(const interface::msg::RobotState::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Rx:ret_robot_state=%d", msg->value);

        robot_state_ = *msg;
    }

    void RxBattery(const interface::msg::BatteryInfo::SharedPtr msg) { battery_info_ = *msg; }

    void RxSmartFollow(const interface::msg::SmartFollow::SharedPtr msg)
    {
        smart_follow_x_ = msg->x[0];
        smart_follow_y_ = msg->y[0];
        RCLCPP_DEBUG(this->get_logger(), "smart_follow: x=%f, y=%f", smart_follow_x_, smart_follow_y_);
    }

    void RxTrigger(const std_srvs::srv::Trigger::Request::SharedPtr request, std_srvs::srv::Trigger::Response::SharedPtr response)
    {
        (void)request;
        RCLCPP_DEBUG(this->get_logger(), "/key_control/bt_trigger");
        response->success = GamepadScan();

        response->message = "trigger_key_control";
    }

private:
    // 定时器
    rclcpp::TimerBase::SharedPtr timerRead_;
    rclcpp::TimerBase::SharedPtr timerCheck_; // 检查通断

    // topic发布端
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_emerg_stop_;
    // topic接受端

    // service发布端

    // service接收端

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_vel_; // 线速度角速度
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_vel_;

    rclcpp::Subscription<interface::msg::Pose>::SharedPtr sub_pos_; // 姿态
    rclcpp::Publisher<interface::msg::Pose>::SharedPtr pub_pos_;

    rclcpp::Subscription<interface::msg::QuadState>::SharedPtr sub_qr_state_; // 四足特有状态
    rclcpp::Publisher<interface::msg::QuadState>::SharedPtr pub_qr_state_;

    rclcpp::Subscription<interface::msg::RunState>::SharedPtr sub_run_state_;
    rclcpp::Publisher<interface::msg::RunState>::SharedPtr pub_run_state_;

    rclcpp::Subscription<interface::msg::WalkMode>::SharedPtr sub_walk_mode_;
    rclcpp::Publisher<interface::msg::WalkMode>::SharedPtr pub_walk_mode_;

    rclcpp::Subscription<interface::msg::BatteryInfo>::SharedPtr sub_battery_;    // 电池
    rclcpp::Subscription<interface::msg::RobotState>::SharedPtr sub_robot_state_; // 机器人状态

    rclcpp::Client<interface::srv::DeviceInfo>::SharedPtr cli_device_info_;                // 机器人信息
    rclcpp::Client<interface::srv::EnableSpeechRecog>::SharedPtr cli_enable_speech_recog_; // 语音识别节点使能开关

    rclcpp::Subscription<interface::msg::SmartFollow>::SharedPtr sub_smart_follow_;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_open_smart_follow_; // 通知智能跟随打开关闭

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _tts_content_pub_ptr; // 给语音播报节点发送内容

    // 客户端回调组
    rclcpp::CallbackGroup::SharedPtr client_cb_group_;
    rclcpp::Client<interface::srv::Dance>::SharedPtr _dance_cli_ptr;
    // 服务端回调组
    rclcpp::CallbackGroup::SharedPtr service_cb_group_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_trigger_; // 触发器，临时，若手柄存在/有数据，则返回true，否则返回false

    // 旧数据
    geometry_msgs::msg::Twist vel_;
    interface::msg::Pose pos_;
    interface::msg::QuadState qr_sta_;
    interface::msg::RunState run_sta_;
    interface::msg::WalkMode walk_mode_;
    interface::msg::BatteryInfo battery_info_;
    interface::msg::RobotState robot_state_;
    double smart_follow_x_, smart_follow_y_;
    bool is_smart_follow_{false};
    bool is_key_exist_{false};

    KeyBase key_;
    std::string js_;
    bool is_exist_{false};
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<KeyQuadruped>();
    RCLCPP_INFO(node->get_logger(), "Hello key quadruped!");
    executor.add_node(node);
    executor.spin();
    // rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
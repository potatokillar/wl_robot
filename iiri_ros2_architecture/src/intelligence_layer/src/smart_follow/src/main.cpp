/*
 * @Author: 唐文浩
 * @Date: 2024-12-13
 * @LastEditors: 唐文浩
 * @LastEditTime: 2025-07-04
 * @Description: 智能跟随数据读取发布节点
 *
 * Copyright (c) 2022-2024 by 西湖大学智能产业研究院, All Rights Reserved.
 */
#include <deque>
#include <rclcpp/rclcpp.hpp>

#include "geometry_msgs/msg/twist.hpp"
#include "interface/msg/gamepad_cmd.hpp"
#include "robot_base/cpp_types.hpp"
#include "robot_base/timer_tools.hpp"
#include "serialPort.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

class SmartFollowSensor : public rclcpp::Node
{
public:
    SmartFollowSensor() : Node("smart_follow_sensor")
    {
        pub_vel_ = create_publisher<geometry_msgs::msg::Twist>("/motion_control/cmd_vel", 10);
        srv_trigger_ = create_service<std_srvs::srv::Trigger>(
            "/smart_follow/bt_trigger",
            [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response)
            {
                this->RxTrigger(request, response);
            });

        sub_open_ = create_subscription<interface::msg::GamepadCmd>("/motion_control/ret_gamepad_cmd",
                                                                    10,
                                                                    [this](const interface::msg::GamepadCmd::SharedPtr msg)
                                                                    { this->RxOpen(msg); });
        sub_topic_cmd_ =
            create_subscription<std_msgs::msg::Bool>("/smart_follow/open", 10, [this](const std_msgs::msg::Bool::SharedPtr msg)
                                                     { this->RxTopicCmd(msg); });
        sub_xiaozhi_stt_ =
            create_subscription<std_msgs::msg::String>("/xiaozhi/stt", 10, [this](const std_msgs::msg::String::SharedPtr msg)
                                                       { this->RxXiaozhiStt(msg); });
        // 设备有几率识别到usb1扣上
        if (GetSerialPort().Open("smart_follow", "/dev/ttyUSB0"))
        {
            if (GetSerialPort().SetParam("smart_follow", 115200, 8, Parity::none, StopBits::one))
            {
                GetSerialPort().AsyncRead("smart_follow", [this](const std::vector<uint8_t> &data)
                                          { this->RecvDeal(data); });
                RCLCPP_INFO(this->get_logger(), "smart_follow_sensor: serial port open success!");
            }
        }
        else if (GetSerialPort().Open("smart_follow", "/dev/ttyUSB1"))
        {
            if (GetSerialPort().SetParam("smart_follow", 115200, 8, Parity::none, StopBits::one))
            {
                GetSerialPort().AsyncRead("smart_follow", [this](const std::vector<uint8_t> &data)
                                          { this->RecvDeal(data); });
                RCLCPP_INFO(this->get_logger(), "smart_follow_sensor: serial port open success!");
            }
        }
    }

private:
    // 解析数据，传进来的数据已经保证了数据头和数据长度的正确性
    void ParseData(const std::string &oneData)
    {
        // MP0042,0,-145,46,206,214,-151.74,-52.53,0,0,0,80
        // 根据逗号分割字符串
        std::vector<std::string> parts;
        std::stringstream ss(oneData);
        std::string part;
        while (std::getline(ss, part, ','))
        {
            parts.push_back(part);
        }

        x_ = std::stoi(parts[2]) / 100.0;
        y_ = std::stoi(parts[3]) / 100.0;
        //  pub_->publish(msg);
        // RCLCPP_DEBUG(this->get_logger(), "smart follow, x:%f, y:%f", msg.x[0], msg.y[0]);
    }

    void RecvDeal(const std::vector<uint8_t> &data)
    {
        for (const auto &v : data)
        {
            rxData_.push_back(v);
        }

        std::string oneData;
        while (rxData_.size() > 6)
        {
            if (rxData_.at(0) == 'M' && rxData_.at(1) == 'P')
            {
                size_t num = 0;
                for (int i = 0; i < 4; i++)
                {
                    num = num * 10 + (rxData_.at(i + 2) - '0');
                }

                // 剩余数据长度符合要求
                if (num < rxData_.size() - 6)
                {
                    oneData.append(rxData_.begin(), rxData_.begin() + num + 6);
                    rxData_.erase(rxData_.begin(), rxData_.begin() + num + 6);
                    ParseData(oneData);
                }
                else
                {
                    break; // 不符合要求则退出循环，等待下一段数据再解析
                }
            }
            else
            {
                rxData_.pop_front();
            }
        }
    }

    void Send2MotionControl()
    {
        geometry_msgs::msg::Twist vel;
        double px = -x_; // 10ms
        double py = y_;

        double kz = 1.5;
        double xCmd = 0.0;
        vel.angular.z = kz * (px - xCmd);
        double kx = 1.3;
        double yCmd = 1.0;
        if (fabs(py) <= 0.01)
        {
            vel.linear.x = 0;
        }
        else
        {
            vel.linear.x = kx * (py - yCmd);
        }

        vel.angular.x = 0.0;
        vel.angular.y = 0.0;
        vel.linear.y = 0.0;
        vel.linear.z = 0.0;
        pub_vel_->publish(vel);
    }

    void RxTrigger(const std_srvs::srv::Trigger::Request::SharedPtr request, std_srvs::srv::Trigger::Response::SharedPtr response)
    {
        (void)request;
        // RCLCPP_INFO(this->get_logger(), "/smart_follow/bt_trigger");
        response->success = isRunning_;
        if (isRunning_)
        {
            Send2MotionControl();
        }
    }

    void RxOpen(const interface::msg::GamepadCmd::SharedPtr msg)
    {
        // 按下切换，释放一次后重置
        // 若不是Y按键，则退出智能跟随
        if ((msg->y) && (msg->y != last_key_))
        {
            isRunning_ = !isRunning_;
            RCLCPP_INFO(this->get_logger(), "Rx:open:%d", isRunning_);
        }
        else if ((last_takeover_ == false) && (msg->takeover == true) && (isRunning_ == true))
        {
            isRunning_ = false;
            RCLCPP_INFO(this->get_logger(), "gamepad takeover, stop! %d", isRunning_);
        }

        last_takeover_ = msg->takeover;
        last_key_ = msg->y;
    }

    void RxTopicCmd(const std_msgs::msg::Bool::SharedPtr msg)
    {
        isRunning_ = msg->data;
        RCLCPP_INFO(this->get_logger(), "Rx:open:%d by topic from other node", isRunning_);
    }

    void RxXiaozhiStt(const std_msgs::msg::String::SharedPtr msg)
    {
        std::string stt_result = msg->data;
        for (const auto &item : _open_list)
        {
            if (stt_result.find(item) != std::string::npos)
            {
                isRunning_ = true;
                RCLCPP_INFO(this->get_logger(), "Rx:open:1 by /xiaozhi/stt");
                break;
                return;
            }
        }
        for (const auto &item : _stop_list)
        {
            if (stt_result.find(item) != std::string::npos)
            {
                isRunning_ = false;
                RCLCPP_INFO(this->get_logger(), "Rx:open:0 by /xiaozhi/stt");
                break;
                return;
            }
        }
    }

private:
    std::deque<char> rxData_;
    double x_ = 0.0;
    double y_ = 0.0;
    bool isRunning_ = false;
    bool last_key_ = false;
    bool ignore_takeover_ = false;
    bool last_takeover_ = false;
    std::vector<std::string> _open_list = {"进入跟随模式",
                                           "进入智能跟随",
                                           "进入智能跟随",
                                           "开启跟随模式",
                                           "开启智能跟随",
                                           "开启只能跟随",
                                           "打开跟随模式",
                                           "打开智能跟随",
                                           "打开只能跟随",
                                           "跟着我"};
    std::vector<std::string> _stop_list = {"退出跟随模式",
                                           "退出智能跟随",
                                           "退出只能跟随",
                                           "停止跟随模式",
                                           "停止智能跟随",
                                           "停止只能跟随",
                                           "关闭跟随模式",
                                           "关闭智能跟随",
                                           "关闭只能跟随",
                                           "不用跟随",
                                           "不用跟着"};
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_vel_;
    rclcpp::Subscription<interface::msg::GamepadCmd>::SharedPtr sub_open_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_topic_cmd_;     // 通过其他节点发来的开启智能跟随的topic
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_xiaozhi_stt_; // 接收xiaozhi节点发来的语音识别指令
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_trigger_;         // 触发器，临时，若手柄存在/有数据，则返回true，否则返回false
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SmartFollowSensor>();
    RCLCPP_INFO(node->get_logger(), "Hello smart follow node");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
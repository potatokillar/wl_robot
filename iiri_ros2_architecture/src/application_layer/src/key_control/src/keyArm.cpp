/*
 * @Author: 唐文浩
 * @Date: 2024-11-06
 * @LastEditors: 唐文浩
 * @LastEditTime: 2025-02-11
 * @Description: 机械臂按键部分，机械臂按键暂不被行为树管理，只发给行为树任务
 *
 * Copyright (c) 2022-2024 by 西湖大学智能产业研究院, All Rights Reserved.
 */

#include "interface/msg/arm_state.hpp"
#include "interface/srv/arm_cmd.hpp"
#include "interface/srv/exec_tree.hpp"
#include "interface/srv/move_j.hpp"
#include "interface/srv/move_l.hpp"
#include "keyBase.hpp"
#include "rclcpp/rclcpp.hpp"
#include "timerTools.hpp"

class KeyArm : public rclcpp::Node
{
public:
    KeyArm() : Node("key_arm")
    {
        flag_init_last_state_ = false;

        this->declare_parameter<std::string>("js", "/dev/input/js0");
        js_ = this->get_parameter("js").as_string();
        RCLCPP_INFO(this->get_logger(), "js: %s", js_.c_str());

        sub_arm_state_ =
            this->create_subscription<interface::msg::ArmState>("/motion_control/arm_state", 10, [this](const interface::msg::ArmState::SharedPtr msg)
                                                                {
                now_state_.cart = msg->cart;
                now_state_.angle = msg->angle;
                now_state_.state = msg->state;
                if (!flag_init_last_state_) {
                    last_state_cmd_.cart = msg->cart;
                    last_state_cmd_.angle = msg->angle;
                    last_state_cmd_.state = msg->state;
                    flag_init_last_state_ = true;
                } });

        cli_exec_tree_ = this->create_client<interface::srv::ExecTree>("/bt_manager/exec_tree");
        cli_move_j_ = this->create_client<interface::srv::MoveJ>("/motion_control/move_j");
        cli_move_l_ = this->create_client<interface::srv::MoveL>("/motion_control/move_l");
        cli_cmd_ = this->create_client<interface::srv::ArmCmd>("/motion_control/arm_cmd");

        if (cli_move_j_->wait_for_service(std::chrono::seconds(5)) == false)
        {
            RCLCPP_ERROR(this->get_logger(), "move_j service not ready");
        }

        if (cli_cmd_->wait_for_service(std::chrono::seconds(5)) == false)
        {
            RCLCPP_ERROR(this->get_logger(), "arm_cmd service not ready");
        }

        thread_ = std::thread(&KeyArm::BlockRun, this);
    }

    void BlockRun()
    {
        while (rclcpp::ok())
        {
            auto jsData = GamepadRead(js_);
            if (jsData.has_value())
            {
                key_.SaveGamepadBtn(jsData.value());
            }
            else
            {
                TimerTools::SleepForMs(10);
                continue;
            }
            if (key_.IsShortRelease("y"))
            {
                if (mode_ != 0)
                {
                    RCLCPP_INFO(this->get_logger(), "\n");
                    RCLCPP_INFO(this->get_logger(), "-------------------");
                    RCLCPP_INFO(this->get_logger(), "选择 Task 模式 !!!");
                    mode_ = 0;
                }
                else
                {
                    RCLCPP_INFO(this->get_logger(), "\n");
                    RCLCPP_INFO(this->get_logger(), "-------------------");
                    RCLCPP_INFO(this->get_logger(), "选择 MoveL 模式 !!!");
                    mode_ = 1;
                    to_movel_start();
                }
            }
            if (key_.IsShortRelease("lt"))
            {
                RCLCPP_INFO(this->get_logger(), "\n");
                RCLCPP_INFO(this->get_logger(), "-------------------");
                RCLCPP_INFO(this->get_logger(), "选择 MoveJ 模式 !!!");
                mode_ = 2;
                to_movej_start();
            }

            bool is_ok = false;
            auto request = std::make_shared<interface::srv::ArmCmd::Request>();
            if (key_.IsShortRelease("start"))
            {
                request->cmd = interface::srv::ArmCmd::Request::ENABLE;
                is_ok = true;
            }
            if (key_.IsShortRelease("back"))
            {
                request->cmd = interface::srv::ArmCmd::Request::DISABLE;
                is_ok = true;
            }

            if (is_ok)
            {
                cli_cmd_->async_send_request(request, [this](rclcpp::Client<interface::srv::ArmCmd>::SharedFuture future)
                                             { auto response = future.get(); });
            }
            switch (mode_)
            {
            case 0:
                JoyTask();
                break;
            case 1:
                JoyMoveL();
                break;
            case 2:
                JoyMoveJ();
                break;

            default:
                break;
            }

            TimerTools::SleepForMs(50);
        }
    }

    void JoyTask()
    {
        bool new_task = false;
        auto request = std::make_shared<interface::srv::ExecTree::Request>();
        if (key_.IsShortRelease("x"))
        { // 准备位置
            request->tree = "arm_qa01_open_door";
            new_task = true;
        }

        if (key_.IsShortRelease("a"))
        {
            request->tree = "arm_qa01_catch_over_table";
            new_task = true;
        }

        if (key_.IsShortRelease("b"))
        {
            request->tree = "arm_qa01_catch_down_table";
            new_task = true;
        }

        if (new_task)
        {
            auto future = cli_exec_tree_->async_send_request(request);
            if (future.get()->success)
            {
                RCLCPP_INFO(this->get_logger(), "ExecTree success:%s", request->tree.c_str());
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "ExecTree failed:%s", request->tree.c_str());
            }
        }
    }

    void JoyMoveL()
    {
        auto request = std::make_shared<interface::srv::MoveL::Request>();
        request->tran.x = last_state_cmd_.cart.x;
        request->tran.y = last_state_cmd_.cart.y;
        request->tran.z = last_state_cmd_.cart.z;
        request->rpy.x = last_state_cmd_.cart.roll;
        request->rpy.y = last_state_cmd_.cart.pitch;
        request->rpy.z = last_state_cmd_.cart.yaw;
        bool is_valid_key = false;

        if (key_.IsShortRelease("up"))
        {
            request->tran.z += 0.1;
            is_valid_key = true;
        }
        if (key_.IsShortRelease("down"))
        {
            request->tran.z -= 0.1;
            is_valid_key = true;
        }
        if (key_.IsShortRelease("left"))
        {
            request->tran.y += 0.1;
            is_valid_key = true;
        }
        if (key_.IsShortRelease("right"))
        {
            request->tran.y -= 0.1;
            is_valid_key = true;
        }

        if (key_.GetStick().at("ly") > 0.5)
        {
            request->tran.x -= 0.1;
            is_valid_key = true;
        }
        if (key_.GetStick().at("ly") < -0.5)
        {
            request->tran.x += 0.1;
            is_valid_key = true;
        }

        if (key_.GetStick().at("lx") > 0.5)
        {
            request->rpy.x += 0.01;
            is_valid_key = true;
        }
        if (key_.GetStick().at("lx") < -0.5)
        {
            request->rpy.x -= 0.01;
            is_valid_key = true;
        }

        if (key_.GetStick().at("ry") > 0.5)
        {
            request->rpy.y += 0.01;
            is_valid_key = true;
        }
        if (key_.GetStick().at("ry") < -0.5)
        {
            request->rpy.y -= 0.01;
            is_valid_key = true;
        }

        if (key_.GetStick().at("rx") > 0.5)
        {
            request->rpy.z += 0.01;
            is_valid_key = true;
        }
        if (key_.GetStick().at("rx") < -0.5)
        {
            request->rpy.z -= 0.01;
            is_valid_key = true;
        }

        request->mode = "abs";
        request->speed = 0;
        request->acc = 0;
        // 如果没有检测到有效按键值或者没有从上一次读取到上一次理想关节角的话，就直接返回不发送
        if (!flag_init_last_state_ || !is_valid_key)
            return;
        auto future = cli_move_l_->async_send_request(request);

        if (future.get()->result == 0)
        {
            last_state_cmd_.cart.x = request->tran.x;
            last_state_cmd_.cart.y = request->tran.y;
            last_state_cmd_.cart.z = request->tran.z;
            last_state_cmd_.cart.roll = request->rpy.x;
            last_state_cmd_.cart.pitch = request->rpy.y;
            last_state_cmd_.cart.yaw = request->rpy.z;
            RCLCPP_INFO(this->get_logger(), "SUCCESS!!! movel joint");
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "FAILURE!!! movel joint");
        }
    }

    void JoyMoveJ()
    {
        auto request = std::make_shared<interface::srv::MoveJ::Request>();
        for (int i = 0; i < 6; i++)
        {
            request->joint.at(i) = last_state_cmd_.angle[i];
        }
        bool is_valid_key = false;

        if (key_.GetStick().at("lx") > 0.5)
        {
            request->joint.at(0) += 0.1;
            is_valid_key = true;
        }
        if (key_.GetStick().at("lx") < -0.5)
        {
            request->joint.at(0) -= 0.1;
            is_valid_key = true;
        }

        if (key_.GetStick().at("ly") > 0.5)
        {
            request->joint.at(1) -= 0.3;
            is_valid_key = true;
        }
        if (key_.GetStick().at("ly") < -0.5)
        {
            request->joint.at(1) += 0.3;
            is_valid_key = true;
        }

        if (key_.GetStick().at("ry") > 0.5)
        {
            request->joint.at(2) += 0.3;
            is_valid_key = true;
        }
        if (key_.GetStick().at("ry") < -0.5)
        {
            request->joint.at(2) -= 0.3;
            is_valid_key = true;
        }

        if (key_.GetStick().at("rx") > 0.5)
        {
            request->joint.at(3) += 0.3;
            is_valid_key = true;
        }
        if (key_.GetStick().at("rx") < -0.5)
        {
            request->joint.at(3) -= 0.3;
            is_valid_key = true;
        }

        if (key_.IsShortRelease("up"))
        {
            request->joint.at(4) -= 0.3;
            is_valid_key = true;
        }
        if (key_.IsShortRelease("down"))
        {
            request->joint.at(4) += 0.3;
            is_valid_key = true;
        }

        if (key_.IsShortRelease("left"))
        {
            request->joint.at(5) -= 0.3;
            is_valid_key = true;
        }
        if (key_.IsShortRelease("right"))
        {
            request->joint.at(5) += 0.3;
            is_valid_key = true;
        }

        request->mode = "abs";
        request->speed = 0;
        request->acc = 0;
        // 如果没有检测到有效按键值或者没有从上一次读取到上一次理想关节角的话，就直接返回不发送
        if (!flag_init_last_state_ || !is_valid_key)
            return;
        auto future = cli_move_j_->async_send_request(request);
        if (future.get()->result == 0)
        {
            for (int i = 0; i < 6; i++)
            {
                last_state_cmd_.angle[i] = request->joint.at(i);
            }
            RCLCPP_INFO(this->get_logger(), "SUCCESS!!! movej joint");
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "FAILURE!!! movej joint");
        }
    }

    void to_movej_start()
    {
        { // 只支持qa01，其他机械臂未支持
            auto request = std::make_shared<interface::srv::MoveJ::Request>();
            request->joint.at(0) = 0.0;
            request->joint.at(1) = -1.5;
            request->joint.at(2) = 1.5;
            request->joint.at(3) = 0.0;
            request->joint.at(4) = 0.0;
            request->joint.at(5) = 0.0;
            request->mode = "abs";
            request->speed = 0;
            request->acc = 0;
            auto future = cli_move_j_->async_send_request(request);
            if (future.get()->result == 0)
            {
                RCLCPP_INFO(this->get_logger(), "SUCCESS!!! movej end joint");
                last_state_cmd_.angle[0] = 0.0;
                last_state_cmd_.angle[1] = -1.5;
                last_state_cmd_.angle[2] = 1.5;
                last_state_cmd_.angle[3] = 0.0;
                last_state_cmd_.angle[4] = 0.0;
                last_state_cmd_.angle[5] = 0.0;
                flag_init_last_state_ = true;
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "FAILURE!!! movej end joint");
            }
        }
    }

    void to_movel_start()
    {
        {
            auto request = std::make_shared<interface::srv::MoveL::Request>();
            request->tran.x = 0.5;
            request->tran.y = 0;
            request->tran.z = 0.15;
            request->rpy.x = 3.1416;
            request->rpy.y = 0;
            request->rpy.z = 0;

            request->mode = "abs";
            request->speed = 0;
            request->acc = 0;
            auto future = cli_move_l_->async_send_request(request);
            if (future.get()->result == 0)
            {
                RCLCPP_INFO(this->get_logger(), "SUCCESS!!! init movel pose");
                last_state_cmd_.cart.x = 0.5;
                last_state_cmd_.cart.y = 0;
                last_state_cmd_.cart.z = 0.15;
                last_state_cmd_.cart.roll = 3.1416;
                last_state_cmd_.cart.pitch = 0;
                last_state_cmd_.cart.yaw = 0;
                flag_init_last_state_ = true;
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "FAILURE!!! init movel pose");
            }
        }
    }

private:
    rclcpp::Client<interface::srv::ExecTree>::SharedPtr cli_exec_tree_;
    rclcpp::Client<interface::srv::MoveL>::SharedPtr cli_move_l_;
    rclcpp::Client<interface::srv::MoveJ>::SharedPtr cli_move_j_;
    rclcpp::Client<interface::srv::ArmCmd>::SharedPtr cli_cmd_;

    rclcpp::Subscription<interface::msg::ArmState>::SharedPtr sub_arm_state_;

    std::thread thread_;

    KeyBase key_;
    std::string js_;
    int mode_ = 0; // 0 任务模式，1 move

    interface::msg::ArmState last_state_cmd_;
    bool flag_init_last_state_;

    interface::msg::ArmState now_state_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KeyArm>();
    RCLCPP_INFO(node->get_logger(), "Hello key arm!");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
/*
 * @Author: 唐文浩
 * @Date: 2024-11-01
 * @LastEditors: 唐文浩
 * @LastEditTime: 2025-06-03
 * @Description: 四足控制节点
 *
 * Copyright (c) 2022-2024 by 西湖大学智能产业研究院, All Rights Reserved.
 */

#include "armTask.hpp"
#include "interface/msg/arm_info.hpp"
#include "interface/msg/arm_state.hpp"
#include "interface/msg/io_in_state.hpp"
#include "interface/srv/arm_cmd.hpp"
#include "interface/srv/get_tool_name.hpp"
#include "interface/srv/get_tool_name_list.hpp"
#include "interface/srv/get_tool_value.hpp"
#include "interface/srv/move_j.hpp"
#include "interface/srv/move_l.hpp"
#include "interface/srv/set_gripper.hpp"
#include "interface/srv/set_io_output.hpp"
#include "interface/srv/set_to_home.hpp"
#include "interface/srv/set_to_transport.hpp"
#include "interface/srv/set_tool_name.hpp"
#include "interface/srv/set_tool_value.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std;
using namespace iiri;
using namespace arm;

class arm_node : public rclcpp::Node
{
public:
    arm_node() : Node("arm_node")
    {
        this->declare_parameter<std::string>("ip", "127.0.0.1");
        std::string ip = this->get_parameter("ip").as_string();
        RCLCPP_INFO(this->get_logger(), "ip: %s", ip.c_str());
        ctrl_ = std::make_unique<iiri::arm::ArmTask>(ip);

        timer_recv_ = create_wall_timer(std::chrono::milliseconds(10), [this]()
                                        { this->Run(); });

        pub_arm_sta_ = create_publisher<interface::msg::ArmState>("/motion_control/arm_state", 10);
        pub_arm_info_ = create_publisher<interface::msg::ArmInfo>("/motion_control/arm_info", 10);
        pub_io_in_state_ = create_publisher<interface::msg::IoInState>("/motion_control/io_in_state", 10);

        srv_move_j_ = create_service<interface::srv::MoveJ>(
            "/motion_control/move_j",
            [this](const std::shared_ptr<interface::srv::MoveJ::Request> request, std::shared_ptr<interface::srv::MoveJ::Response> response)
            {
                this->RxMoveJ(request, response);
            });

        srv_move_l_ = create_service<interface::srv::MoveL>(
            "/motion_control/move_l",
            [this](const std::shared_ptr<interface::srv::MoveL::Request> request, std::shared_ptr<interface::srv::MoveL::Response> response)
            {
                this->RxMoveL(request, response);
            });

        srv_cmd_ = create_service<interface::srv::ArmCmd>(
            "/motion_control/arm_cmd",
            [this](const std::shared_ptr<interface::srv::ArmCmd::Request> request, std::shared_ptr<interface::srv::ArmCmd::Response> response)
            {
                this->RxArmCmd(request, response);
            });

        srv_get_tool_name_list_ =
            create_service<interface::srv::GetToolNameList>("/motion_control/get_tool_name_list",
                                                            [this](const std::shared_ptr<interface::srv::GetToolNameList::Request> request,
                                                                   std::shared_ptr<interface::srv::GetToolNameList::Response>
                                                                       response)
                                                            { this->RxGetToolNameList(request, response); });

        srv_get_tool_name_ = create_service<interface::srv::GetToolName>(
            "/motion_control/get_tool_name",
            [this](const std::shared_ptr<interface::srv::GetToolName::Request> request, std::shared_ptr<interface::srv::GetToolName::Response> response)
            {
                this->RxGetToolName(request, response);
            });

        srv_set_tool_name_ = create_service<interface::srv::SetToolName>(
            "/motion_control/set_tool_name",
            [this](const std::shared_ptr<interface::srv::SetToolName::Request> request, std::shared_ptr<interface::srv::SetToolName::Response> response)
            {
                this->RxSetToolName(request, response);
            });

        srv_set_tool_value_ = create_service<interface::srv::SetToolValue>(
            "/motion_control/set_tool_value",
            [this](const std::shared_ptr<interface::srv::SetToolValue::Request> request, std::shared_ptr<interface::srv::SetToolValue::Response> response)
            {
                this->RxSetToolValue(request, response);
            });

        srv_get_tool_value_ = create_service<interface::srv::GetToolValue>(
            "/motion_control/get_tool_value",
            [this](const std::shared_ptr<interface::srv::GetToolValue::Request> request, std::shared_ptr<interface::srv::GetToolValue::Response> response)
            {
                this->RxGetToolValue(request, response);
            });

        srv_set_io_output_ = create_service<interface::srv::SetIoOutput>(
            "/motion_control/set_io_output",
            [this](const std::shared_ptr<interface::srv::SetIoOutput::Request> request, std::shared_ptr<interface::srv::SetIoOutput::Response> response)
            {
                this->RxSetIoOutput(request, response);
            });

        srv_set_to_home_ = create_service<interface::srv::SetToHome>(
            "/motion_control/set_to_home",
            [this](const std::shared_ptr<interface::srv::SetToHome::Request> request, std::shared_ptr<interface::srv::SetToHome::Response> response)
            {
                this->RxSetToHome(request, response);
            });

        srv_set_to_transport_ = create_service<interface::srv::SetToTransport>(
            "/motion_control/set_to_transport",
            [this](const std::shared_ptr<interface::srv::SetToTransport::Request> request, std::shared_ptr<interface::srv::SetToTransport::Response> response)
            {
                this->RxSetToTransport(request, response);
            });
        srv_set_gripper_ = create_service<interface::srv::SetGripper>(
            "/motion_control/set_gripper",
            [this](const std::shared_ptr<interface::srv::SetGripper::Request> request, std::shared_ptr<interface::srv::SetGripper::Response> response)
            {
                this->RxSetGripper(request, response);
            });
        ctrl_->SubscribeNowAngle([this](const std::vector<double> &angle)
                                 { this->RxNowAngle(angle); });
        ctrl_->SubscribeNowCart([this](const CartesianPose &pose)
                                { this->RxNowCart(pose); });
        // ctrl_->SubscribeIoInputState([this](const std::map<std::string, std::bitset<32>>& state) { this->RxIoInputState(state); });
        // ctrl_->SubscribeArmInfo([this](const ArmInfo& info) { this->RxArmInfo(info); });
    }

private:
    void Run() {}
    // sdk注册回调
    void RxNowAngle(const std::vector<double> &angle)
    {
        for (size_t i = 0; i < angle.size() || i < 7; i++)
        {
            state_.angle[i] = angle[i];
        }
        pub_arm_sta_->publish(state_);
    }

    void RxNowCart(const CartesianPose &pose)
    {
        state_.cart.x = pose.x;
        state_.cart.y = pose.y;
        state_.cart.z = pose.z;
        state_.cart.roll = pose.rx;
        state_.cart.pitch = pose.ry;
        state_.cart.yaw = pose.rz;
        pub_arm_sta_->publish(state_);
    }

    void RxIoInputState(const std::map<std::string, std::bitset<32>> &state)
    {
        interface::msg::IoInState ioSta;

        for (const auto &v : state)
        {
            ioSta.name.push_back(v.first);
            ioSta.state.push_back(v.second.to_ulong());
        }

        pub_io_in_state_->publish(ioSta);
    }

    void RxArmInfo(const ArmInfo &info)
    {
        interface::msg::ArmInfo armInfo;

        for (size_t i = 0; i < info.current.size(); i++)
        {
            armInfo.current.push_back(info.current[i]);
            armInfo.temperature.push_back(info.temperature[i]);
            armInfo.torque.push_back(info.torque[i]);
            armInfo.voltage.push_back(info.voltage[i]);
        }
        pub_arm_info_->publish(armInfo);
    }

    // 服务回调
    void RxMoveJ(const std::shared_ptr<interface::srv::MoveJ::Request> request, std::shared_ptr<interface::srv::MoveJ::Response> response)
    {
        MoveMode mode;
        if (request->mode == "abs")
        {
            mode = MoveMode::abs;
        }
        else
        {
            mode = MoveMode::incr;
        }
        std::vector<double> joint;
        for (int i = 0; i < 7; i++)
        {
            joint.push_back(request->joint[i]);
        }
        if (request->speed == 0)
        {
            response->result = Enum2Num(ctrl_->SetMoveJTaskBlock(joint, mode));
        }
        else if (request->acc == 0)
        {
            response->result = Enum2Num(ctrl_->SetMoveJTaskBlock(joint, mode, request->speed));
        }
        else
        {
            response->result = Enum2Num(ctrl_->SetMoveJTaskBlock(joint, mode, request->speed, request->acc));
        }
    }
    void RxMoveL(const std::shared_ptr<interface::srv::MoveL::Request> request, std::shared_ptr<interface::srv::MoveL::Response> response)
    {
        CartesianPose pose;
        pose.x = request->tran.x;
        pose.y = request->tran.y;
        pose.z = request->tran.z;
        pose.rx = request->rpy.x;
        pose.ry = request->rpy.y;
        pose.rz = request->rpy.z;
        MoveMode mode;
        if (request->mode == "abs")
        {
            mode = MoveMode::abs;
        }
        else
        {
            mode = MoveMode::incr;
        }
        if (request->speed == 0)
        {
            response->result = Enum2Num(ctrl_->SetMoveLTaskBlock(pose, mode));
        }
        else if (request->acc == 0)
        {
            response->result = Enum2Num(ctrl_->SetMoveLTaskBlock(pose, mode, request->speed));
        }
        else
        {
            response->result = Enum2Num(ctrl_->SetMoveLTaskBlock(pose, mode, request->speed, request->acc));
        }
    }

    void RxArmCmd(const std::shared_ptr<interface::srv::ArmCmd::Request> request, std::shared_ptr<interface::srv::ArmCmd::Response> response)
    {
        if (request->cmd == interface::srv::ArmCmd::Request::ENABLE)
        {
            response->result = Enum2Num(ctrl_->SetEnable(true));
        }
        else if (request->cmd == interface::srv::ArmCmd::Request::DISABLE)
        {
            response->result = Enum2Num(ctrl_->SetEnable(false));
        }
        else if (request->cmd == interface::srv::ArmCmd::Request::STOP)
        {
            response->result = Enum2Num(ctrl_->SetTaskCmd(ArmTaskCmd::stop));
        }
        else if (request->cmd == interface::srv::ArmCmd::Request::PAUSE)
        {
            response->result = Enum2Num(ctrl_->SetTaskCmd(ArmTaskCmd::pause));
        }
        else if (request->cmd == interface::srv::ArmCmd::Request::START)
        {
            response->result = Enum2Num(ctrl_->SetTaskCmd(ArmTaskCmd::start));
        }
        else
        {
            response->result = Enum2Num(RetState::noSupport);
        }
    }

    void RxGetToolNameList(const std::shared_ptr<interface::srv::GetToolNameList::Request> request,
                           std::shared_ptr<interface::srv::GetToolNameList::Response> response)
    {
        (void)request;
        auto result = ctrl_->GetToolNameList();
        if (result.first == RetState::ok)
        {
            for (auto &name : result.second)
            {
                response->names.push_back(name);
            }
            response->result = Enum2Num(RetState::ok);
        }
        else
        {
            response->result = Enum2Num(result.first);
        }
    }
    void RxGetToolName(const std::shared_ptr<interface::srv::GetToolName::Request> request, std::shared_ptr<interface::srv::GetToolName::Response> response)
    {
        (void)request;
        auto result = ctrl_->GetToolName();
        if (result.first == RetState::ok)
        {
            response->name = result.second;
            response->result = Enum2Num(RetState::ok);
        }
        else
        {
            response->result = Enum2Num(result.first);
        }
    }
    void RxSetToolName(const std::shared_ptr<interface::srv::SetToolName::Request> request, std::shared_ptr<interface::srv::SetToolName::Response> response)
    {
        response->result = Enum2Num(ctrl_->SetToolName(request->name));
    }
    void RxSetToolValue(const std::shared_ptr<interface::srv::SetToolValue::Request> request, std::shared_ptr<interface::srv::SetToolValue::Response> response)
    {
        CartesianPose pose;
        pose.x = request->pose[0];
        pose.y = request->pose[0];
        pose.z = request->pose[0];
        pose.rx = request->pose[0];
        pose.ry = request->pose[0];
        pose.rz = request->pose[0];
        response->result = Enum2Num(ctrl_->SetToolValue(request->name, pose));
    }
    void RxGetToolValue(const std::shared_ptr<interface::srv::GetToolValue::Request> request, std::shared_ptr<interface::srv::GetToolValue::Response> response)
    {
        auto result = ctrl_->GetToolValue(request->name);
        if (result.first == RetState::ok)
        {
            response->pose[0] = result.second.x;
            response->pose[1] = result.second.y;
            response->pose[2] = result.second.z;
            response->pose[3] = result.second.rx;
            response->pose[4] = result.second.ry;
            response->pose[5] = result.second.rz;
            response->result = Enum2Num(RetState::ok);
        }
        else
        {
            response->result = Enum2Num(result.first);
        }
    }
    void RxSetIoOutput(const std::shared_ptr<interface::srv::SetIoOutput::Request> request, std::shared_ptr<interface::srv::SetIoOutput::Response> response)
    {
        response->result = Enum2Num(ctrl_->SetIoOutputState(request->name, request->set));
    }
    void RxSetToHome(const std::shared_ptr<interface::srv::SetToHome::Request> request, std::shared_ptr<interface::srv::SetToHome::Response> response)
    {
        (void)request;
        response->result = Enum2Num(ctrl_->SetToHome());
    }
    void RxSetToTransport(const std::shared_ptr<interface::srv::SetToTransport::Request> request,
                          std::shared_ptr<interface::srv::SetToTransport::Response> response)
    {
        (void)request;
        response->result = Enum2Num(ctrl_->SetToTransport());
    }
    void RxSetGripper(const std::shared_ptr<interface::srv::SetGripper::Request> request, std::shared_ptr<interface::srv::SetGripper::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "SetGripper:%d", request->state);
        response->result = Enum2Num(ctrl_->SetGripper(request->state, request->speed));
    }

private:
    std::unique_ptr<iiri::arm::ArmTask> ctrl_;

    rclcpp::TimerBase::SharedPtr timer_recv_;

    // 接收服务
    rclcpp::Service<interface::srv::MoveJ>::SharedPtr srv_move_j_;
    rclcpp::Service<interface::srv::MoveL>::SharedPtr srv_move_l_;
    rclcpp::Service<interface::srv::ArmCmd>::SharedPtr srv_cmd_;
    rclcpp::Service<interface::srv::GetToolNameList>::SharedPtr srv_get_tool_name_list_;
    rclcpp::Service<interface::srv::GetToolName>::SharedPtr srv_get_tool_name_;
    rclcpp::Service<interface::srv::SetToolName>::SharedPtr srv_set_tool_name_;
    rclcpp::Service<interface::srv::SetToolValue>::SharedPtr srv_set_tool_value_;
    rclcpp::Service<interface::srv::GetToolValue>::SharedPtr srv_get_tool_value_;
    rclcpp::Service<interface::srv::SetIoOutput>::SharedPtr srv_set_io_output_;
    rclcpp::Service<interface::srv::SetToHome>::SharedPtr srv_set_to_home_;
    rclcpp::Service<interface::srv::SetToTransport>::SharedPtr srv_set_to_transport_;
    rclcpp::Service<interface::srv::SetGripper>::SharedPtr srv_set_gripper_;

    // 自动上报信息
    rclcpp::Publisher<interface::msg::ArmState>::SharedPtr pub_arm_sta_;
    rclcpp::Publisher<interface::msg::ArmInfo>::SharedPtr pub_arm_info_;
    rclcpp::Publisher<interface::msg::IoInState>::SharedPtr pub_io_in_state_;

private:
    // 返回值
    interface::msg::ArmState state_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<arm_node>();
    RCLCPP_INFO(node->get_logger(), "Hello arm_node!");
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
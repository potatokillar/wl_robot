/*
 * @Author: 唐文浩
 * @Date: 2024-11-01
 * @LastEditors: 唐文浩
 * @LastEditTime: 2025-07-01
 * @Description: 四足控制节点
 *
 * Copyright (c) 2022-2024 by 西湖大学智能产业研究院, All Rights Reserved.
 */
#include "qr.hpp"

#include <std_msgs/msg/float32.hpp>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "interface/msg/battery_info.hpp"
#include "interface/msg/gamepad_cmd.hpp"
#include "interface/msg/pose.hpp"
#include "interface/msg/quad_state.hpp"
#include "interface/msg/robot_state.hpp"
#include "interface/msg/run_state.hpp"
#include "interface/msg/walk_mode.hpp"
#include "interface/srv/dance.hpp"
#include "interface/srv/device_info.hpp"
#include "interface/srv/dog_move.hpp"
#include "quadruped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_base/timer_tools.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/bool.hpp"

qr_node::qr_node() : Node("qr_node")
{
    this->declare_parameter<std::string>("ip", "127.0.0.1");
    std::string ip = this->get_parameter("ip").as_string();
    RCLCPP_INFO(this->get_logger(), "ip: %s", ip.c_str());

    //  timer_recv_ = create_wall_timer(std::chrono::milliseconds(100), [this]() { this->Run(); });

    // 接收其他node的数据
    sub_vel_ = create_subscription<geometry_msgs::msg::Twist>("/motion_control/cmd_vel", 10, [this](const geometry_msgs::msg::Twist::SharedPtr msg)
                                                              { this->RxVel(msg); });
    sub_vel_ratio_ = create_subscription<geometry_msgs::msg::Twist>("/motion_control/cmd_vel_ratio",
                                                                    10,
                                                                    [this](const geometry_msgs::msg::Twist::SharedPtr msg)
                                                                    { this->RxVelRatio(msg); });
    sub_pos_ =
        create_subscription<interface::msg::Pose>("/motion_control/cmd_pos", 10, [this](const interface::msg::Pose::SharedPtr msg)
                                                  { this->RxPos(msg); });
    sub_pos_ratio_ = create_subscription<interface::msg::Pose>("/motion_control/cmd_pos_ratio", 10, [this](const interface::msg::Pose::SharedPtr msg)
                                                               { this->RxPosRatio(msg); });
    sub_quad_state_ = create_subscription<interface::msg::QuadState>("/motion_control/cmd_quad_state",
                                                                     10,
                                                                     [this](const interface::msg::QuadState::SharedPtr msg)
                                                                     { this->RxQuadState(msg); });
    sub_walk_mode_ = create_subscription<interface::msg::WalkMode>("/motion_control/cmd_walk_mode", 10, [this](const interface::msg::WalkMode::SharedPtr msg)
                                                                   { this->RxWalkMode(msg); });
    sub_run_state_ = create_subscription<interface::msg::RunState>("/motion_control/cmd_run_state", 10, [this](const interface::msg::RunState::SharedPtr msg)
                                                                   { this->RxSetRunState(msg); });
    sub_height_ = create_subscription<std_msgs::msg::Float32>("/motion_control/cmd_height", 10, [this](const std_msgs::msg::Float32::SharedPtr msg)
                                                              { this->RxHeight(msg); });
    sub_emerg_stop_ = create_subscription<std_msgs::msg::Bool>("/motion_control/cmd_emerg_stop", 10, [this](const std_msgs::msg::Bool::SharedPtr msg)
                                                               { this->RxEmergStop(msg); });

    // 发布给其他node
    pub_vel_ = create_publisher<geometry_msgs::msg::Twist>("/motion_control/ret_vel", 10);
    pub_pos_ = create_publisher<interface::msg::Pose>("/motion_control/ret_pos", 10);
    pub_quad_state_ = create_publisher<interface::msg::QuadState>("/motion_control/ret_quad_state", 10);
    pub_run_state_ = create_publisher<interface::msg::RunState>("/motion_control/ret_run_state", 10);
    pub_walk_mode_ = create_publisher<interface::msg::WalkMode>("/motion_control/ret_walk_mode", 10);
    pub_state_ = create_publisher<interface::msg::RobotState>("/motion_control/ret_robot_state", 10);
    pub_battery_info_ = create_publisher<interface::msg::BatteryInfo>("/motion_control/ret_battery_info", 10);
    pub_height_ = create_publisher<std_msgs::msg::Float32>("/motion_control/ret_height", 10);
    pub_gamepad_cmd_ = create_publisher<interface::msg::GamepadCmd>("/motion_control/ret_gamepad_cmd", 10);

    // 服务接收
    srv_dev_info_ = create_service<interface::srv::DeviceInfo>(
        "/device_info",
        [this](const std::shared_ptr<interface::srv::DeviceInfo::Request> request, std::shared_ptr<interface::srv::DeviceInfo::Response> response)
        {
            this->RxDeviceInfo(request, response);
        });
    srv_dog_move_ = create_service<interface::srv::DogMove>(
        "/dog_move",
        [this](const std::shared_ptr<interface::srv::DogMove::Request> request, std::shared_ptr<interface::srv::DogMove::Response> response)
        {
            this->RxDogMove(request, response);
        });
    srv_dance_ = create_service<interface::srv::Dance>(
        "/dance",
        [this](const std::shared_ptr<interface::srv::Dance::Request> request, std::shared_ptr<interface::srv::Dance::Response> response)
        {
            this->RxDance(request, response);
        });

    quadruped_ = std::make_unique<iiri::qr::Quadruped>(ip);
    quadruped_->SubscribeEvent([this](iiri::RobotEvent event)
                               {
        if (event == iiri::RobotEvent::connect) {
            this->sdk_connect();
        } });
}

void qr_node::sdk_connect()
{
    RCLCPP_INFO(this->get_logger(), "sdk_connect success");
    quadruped_->SubscribeRunState([this](iiri::qr::RunState state)
                                  { this->PubRunState(state); });
    quadruped_->SubscribeWalkMode([this](iiri::qr::WalkMode mode)
                                  { this->PubWalkMode(mode); });
    quadruped_->SubscribeLinearVelocity([this](const iiri::qr::LinearVelocityData &v)
                                        { this->PubLinearVelocity(v); });
    quadruped_->SubscribeAngularVelocity([this](const iiri::qr::AngularVelocityData &v)
                                         { this->PubAngularVelocity(v); });
    quadruped_->SubscribePose([this](const iiri::qr::PoseData &p)
                              { this->PubPose(p); });
    quadruped_->SubscribeBatteryInfo([this](const iiri::BatteryInfo &info)
                                     { this->PubBatteryInfo(info); });
    quadruped_->SubscribeHeight([this](float height)
                                { this->PubHeight(height); });
    quadruped_->SubscribeGamepadCmd([this](const iiri::GamepadCmd &data)
                                    { this->PubGamepadCmd(data); });
}

void qr_node::RxVel(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    // RCLCPP_INFO(this->get_logger(), "Received message: linear.x=%f, angular.z=%f", msg->linear.x, msg->angular.z);

    quadruped_->SetLinearVelocity(msg->linear.x, msg->linear.y, msg->linear.z);
    quadruped_->SetAngularVelocity(msg->angular.x, msg->angular.y, msg->angular.z);

    RCLCPP_INFO(this->get_logger(), "cmd_vel.linear: %f, %f, %f", msg->linear.x, msg->linear.y, msg->linear.z);
    RCLCPP_INFO(this->get_logger(), "cmd_vel.angular: %f, %f, %f", msg->angular.x, msg->angular.y, msg->angular.z);
}

void qr_node::RxVelRatio(const geometry_msgs::msg::Twist::SharedPtr msg) { (void)msg; }

void qr_node::RxPos(const interface::msg::Pose::SharedPtr msg)
{
    // RCLCPP_INFO(this->get_logger(), "Received message: x=%f, y=%f, z=%f", msg->x, msg->y, msg->z);
    quadruped_->SetPose(msg->roll, msg->pitch, msg->yaw);
}

void qr_node::RxPosRatio(const interface::msg::Pose::SharedPtr msg) { (void)msg; }

/**
 * @description: 四足机器人状态，可以被请求改变
 * @param msg
 * @return {}
 */
void qr_node::RxQuadState(const interface::msg::QuadState::SharedPtr msg)
{
    if ((msg->height != nowQuadState_.height) && (quadruped_->SetHeight(msg->height) == iiri::RetState::ok))
    {
        nowQuadState_.height = msg->height;
    }

    if ((msg->load_mass != nowQuadState_.load_mass) && (quadruped_->SetLoadMass(msg->load_mass) == iiri::RetState::ok))
    {
        nowQuadState_.load_mass = msg->load_mass;
    }
}

void qr_node::RxWalkMode(const interface::msg::WalkMode::SharedPtr msg)
{
    iiri::qr::WalkMode tmp;
    switch (msg->value)
    {
    case interface::msg::WalkMode::NORMAL:
        tmp = iiri::qr::WalkMode::aiNormal;
        break;
    case interface::msg::WalkMode::FAST:
        tmp = iiri::qr::WalkMode::aiFast;
        break;

    case interface::msg::WalkMode::CLIMB:
        tmp = iiri::qr::WalkMode::aiClimb;
        break;
    default:
        break;
    }

    if ((tmp != walkMode_) && (quadruped_->SetWalkMode(tmp) == iiri::RetState::ok))
    {
        walkMode_ = tmp;
    }
}

void qr_node::RxSetRunState(const interface::msg::RunState::SharedPtr msg)
{
    const char *state_name = "UNKNOWN";
    iiri::qr::RunState tmp;
    switch (msg->value)
    {
    case interface::msg::RunState::STAND:
        tmp = iiri::qr::RunState::stand;
        state_name = "STAND";
        break;
    case interface::msg::RunState::WALK:
        tmp = iiri::qr::RunState::walk;
        state_name = "WALK";
        break;
    case interface::msg::RunState::LIE:
        tmp = iiri::qr::RunState::lie;
        state_name = "LIE";
        break;
    default:
        break;
    }

    RCLCPP_INFO(this->get_logger(), "[MOTION_CTRL] ROS2 received run_state command: %s (value=%d)", state_name, msg->value);

    // Query actual state from QR instead of relying on cached state
    // This ensures we detect state changes made by other sources (e.g., web frontend)
    auto [get_ret, actual_state] = quadruped_->GetRunState();

    std::string actual_state_name = "UNKNOWN";
    if (get_ret == iiri::RetState::ok)
    {
        switch (actual_state)
        {
        case iiri::qr::RunState::stand:
            actual_state_name = "STAND";
            break;
        case iiri::qr::RunState::walk:
            actual_state_name = "WALK";
            break;
        case iiri::qr::RunState::lie:
            actual_state_name = "LIE";
            break;
        default:
            actual_state_name = "UNKNOWN";
        }
        RCLCPP_DEBUG(this->get_logger(), "[MOTION_CTRL] Current QR state: %s, Requested: %s", actual_state_name.c_str(), state_name);
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "[MOTION_CTRL] Failed to query current state: RetState=%d", static_cast<int>(get_ret));
    }

    // Compare with desired state and execute if different
    if (tmp != actual_state)
    {
        RCLCPP_INFO(this->get_logger(), "[MOTION_CTRL] State change detected: %s -> %s, calling SDK SetRunState...",
                    actual_state_name.c_str(), state_name);
        auto ret = quadruped_->SetRunState(tmp);
        if (ret == iiri::RetState::ok)
        {
            runState_ = tmp; // Update cache for subscribers
            RCLCPP_INFO(this->get_logger(), "[MOTION_CTRL] SDK SetRunState SUCCESS: %s", state_name);
        }
        else
        {
            // Enhanced error logging with detailed error code
            std::string error_desc = "UNKNOWN";
            switch (ret)
            {
            case iiri::RetState::netErr:
                error_desc = "netErr (网络错误)";
                break;
            case iiri::RetState::outRange:
                error_desc = "outRange (超范围)";
                break;
            case iiri::RetState::timeout:
                error_desc = "timeout (超时)";
                break;
            case iiri::RetState::noSupport:
                error_desc = "noSupport (不支持)";
                break;
            case iiri::RetState::parseErr:
                error_desc = "parseErr (解析错误)";
                break;
            case iiri::RetState::busy:
                error_desc = "busy (设备忙)";
                break;
            case iiri::RetState::interrupt:
                error_desc = "interrupt (任务中断)";
                break;
            case iiri::RetState::error:
                error_desc = "error (设备状态错误/电机故障)";
                break;
            default:
                error_desc = "other error";
            }
            RCLCPP_ERROR(this->get_logger(), "[MOTION_CTRL] SDK SetRunState FAILED: %s -> %s (RetState=%d)",
                         state_name, error_desc.c_str(), static_cast<int>(ret));
        }
    }
    else
    {
        RCLCPP_DEBUG(this->get_logger(), "[MOTION_CTRL] Already in %s state, no change needed", state_name);
    }
}

void qr_node::RxDeviceInfo(const std::shared_ptr<interface::srv::DeviceInfo::Request> request, std::shared_ptr<interface::srv::DeviceInfo::Response> response)
{
    (void)request;
    auto [ret, info] = quadruped_->GetDeviceInfo();
    if (ret == iiri::RetState::ok)
    {
        response->success = true;
        response->name = info.name;
        response->version_hardware = info.version.hardware;
        response->version_software = info.version.software;
        response->version_sdk = info.version.sdk;
        response->version_mechanical = info.version.mechanical;
        response->serial_number = info.serialNo;
        response->model = info.model;
    }
    else
    {
        response->success = false;
    }
}

void qr_node::RxDogMove(const std::shared_ptr<interface::srv::DogMove::Request> request, std::shared_ptr<interface::srv::DogMove::Response> response)
{
    iiri::qr::RunState current_run_state;
    if (auto retSta = quadruped_->GetRunState(); retSta.first == iiri::RetState::ok)
    {
        current_run_state = retSta.second;
    }
    // 先检测状态，如果是lie状态则先切换到stand，然后再切换到walk
    // 阻塞式切换，如果没切换成功则一直等待直至切换完成
    if (current_run_state == iiri::qr::RunState::lie)
    {
        quadruped_->SetRunState(iiri::qr::RunState::stand);
        do
        {
            if (auto retSta = quadruped_->GetRunState(); retSta.first == iiri::RetState::ok)
            {
                current_run_state = retSta.second;
            }
        } while (current_run_state != iiri::qr::RunState::stand);
    }
    if (current_run_state == iiri::qr::RunState::stand)
    {
        quadruped_->SetRunState(iiri::qr::RunState::walk);
        do
        {
            if (auto retSta = quadruped_->GetRunState(); retSta.first == iiri::RetState::ok)
            {
                current_run_state = retSta.second;
            }
        } while (current_run_state != iiri::qr::RunState::walk);
    }

    auto period = request->ms;
    auto lasting = 0;
    auto interval = 20;
    // 要持续发送一段时间，再置为0
    while (lasting < period)
    {
        quadruped_->SetLinearVelocity(request->vel.linear.x, request->vel.linear.y, request->vel.linear.z);
        quadruped_->SetAngularVelocity(request->vel.angular.x, request->vel.angular.y, request->vel.angular.z);
        std::this_thread::sleep_for(std::chrono::milliseconds(interval));
        lasting += interval;
    }
    quadruped_->SetLinearVelocity(0, 0, 0);
    quadruped_->SetAngularVelocity(0, 0, 0);
    response->result = 0;
}

void qr_node::RxDance(const std::shared_ptr<interface::srv::Dance::Request> request, std::shared_ptr<interface::srv::Dance::Response> response)
{
    iiri::qr::RunState current_run_state;
    auto dance_name = request->name;
    if (auto retSta = quadruped_->GetRunState(); retSta.first == iiri::RetState::ok)
    {
        current_run_state = retSta.second;
        // 先检测状态，如果不是stand就设置为stand
        if (current_run_state != iiri::qr::RunState::stand)
        {
            quadruped_->SetRunState(iiri::qr::RunState::stand);
            // 等待状态切换结束
            do
            {
                if (auto retSta = quadruped_->GetRunState(); retSta.first == iiri::RetState::ok)
                {
                    current_run_state = retSta.second;
                }
            } while (current_run_state != iiri::qr::RunState::stand);
        }

        if (quadruped_->SetDance(dance_name) == iiri::RetState::ok)
        {
            response->success = true;
        }
        else
        {
            response->success = false;
        }
    }
    else
    {
        response->success = false;
    }
}

void qr_node::RxHeight(const std_msgs::msg::Float32::SharedPtr msg) { quadruped_->SetHeight(msg->data); }

void qr_node::RxEmergStop(const std_msgs::msg::Bool::SharedPtr msg)
{
    const char *status = msg->data ? "TRUE" : "FALSE";
    if (quadruped_->SetEmergStop(msg->data) == iiri::RetState::ok)
    {
        RCLCPP_INFO(this->get_logger(), "set emergency stop %s successfully", status);
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "set emergency stop %s failed...", status);
    }
}

void qr_node::PubRunState(const iiri::qr::RunState &state)
{
    runState_ = state;
    interface::msg::RunState msg;
    switch (runState_)
    {
    case iiri::qr::RunState::lie:
        msg.value = interface::msg::RunState::LIE;
        break;
    case iiri::qr::RunState::stand:
        msg.value = interface::msg::RunState::STAND;
        break;
    case iiri::qr::RunState::switching:
        msg.value = interface::msg::RunState::SWITCHING;
        break;
    case iiri::qr::RunState::walk:
        msg.value = interface::msg::RunState::WALK;
        break;
    default:
        break;
    }
    RCLCPP_INFO(this->get_logger(), "ret RunState: %d", msg.value);
    pub_run_state_->publish(msg);
}

void qr_node::PubWalkMode(const iiri::qr::WalkMode &mode)
{
    walkMode_ = mode;
    interface::msg::WalkMode msg;
    switch (mode)
    {
    case iiri::qr::WalkMode::aiFast:
        msg.value = interface::msg::WalkMode::FAST;
        break;
    case iiri::qr::WalkMode::aiClimb:
        msg.value = interface::msg::WalkMode::CLIMB;
        break;
    case iiri::qr::WalkMode::aiNormal:
        msg.value = interface::msg::WalkMode::NORMAL;
        break;

    default:
        break;
    }
    pub_walk_mode_->publish(msg);
}

void qr_node::PubBatteryInfo(const iiri::BatteryInfo &info)
{
    interface::msg::BatteryInfo msg;
    msg.charge = info.charge;
    msg.current = info.current;
    msg.quantity = info.quantity;
    msg.voltage = info.voltage;
    pub_battery_info_->publish(msg);
}

void qr_node::PubPose(const iiri::qr::PoseData &pose)
{
    interface::msg::Pose msg;
    msg.roll = pose.roll;
    msg.pitch = pose.pitch;
    msg.yaw = pose.yaw;
    pub_pos_->publish(msg);
}

void qr_node::PubHeight(const float &height)
{
    std_msgs::msg::Float32 msg;
    msg.data = height;
    pub_height_->publish(msg);
}

void qr_node::PubLinearVelocity(const iiri::qr::LinearVelocityData &velocity)
{
    nowVel_.linear.x = velocity.x;
    nowVel_.linear.y = velocity.y;
    nowVel_.linear.z = velocity.z;
    pub_vel_->publish(nowVel_);
}

void qr_node::PubAngularVelocity(const iiri::qr::AngularVelocityData &velocity)
{
    nowVel_.angular.x = velocity.x;
    nowVel_.angular.y = velocity.y;
    nowVel_.angular.z = velocity.z;
    pub_vel_->publish(nowVel_);
}

void qr_node::PubGamepadCmd(const iiri::GamepadCmd &cmd)
{
    interface::msg::GamepadCmd msg;
    msg.x = cmd.x;
    msg.y = cmd.y;
    msg.a = cmd.a;
    msg.b = cmd.b;
    msg.lb = cmd.lb;
    msg.rb = cmd.rb;
    msg.start = cmd.start;
    msg.back = cmd.back;
    msg.up = cmd.up;
    msg.down = cmd.down;
    msg.left = cmd.left;
    msg.right = cmd.right;
    msg.lp = cmd.lp;
    msg.rp = cmd.rp;

    msg.lx = cmd.lx;
    msg.ly = cmd.ly;
    msg.rx = cmd.rx;
    msg.ry = cmd.ry;
    msg.lt = cmd.lt;
    msg.rt = cmd.rt;

    msg.takeover = cmd.takeover;
    // RCLCPP_INFO(this->get_logger(), "gamepad takeover %d", msg.takeover);
    pub_gamepad_cmd_->publish(msg);
}

/**
 * @description: 调用sdk存在延时，可能不能用定时器
 * @return {}
 */
void qr_node::Run()
{
    bool is_ok = false;
    auto height = quadruped_->GetHeight();
    if (height.first == iiri::RetState::ok)
    {
        nowQuadState_.height = height.second;
        is_ok = true;
    }

    auto load_mass = quadruped_->GetLoadMass();
    if (load_mass.first == iiri::RetState::ok)
    {
        nowQuadState_.load_mass = load_mass.second;
        is_ok = true;
    }

    if (is_ok)
    {
        pub_quad_state_->publish(nowQuadState_);
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<qr_node>();
    RCLCPP_INFO(node->get_logger(), "Hello qr_node!");
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
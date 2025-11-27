
#pragma once
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include <any>

#include "baseline.hpp"
#include "robot_msgs/MotorCommand.h"
#include "robot_msgs/MotorState.h"

class SimQuadruped
{
public:
    SimQuadruped();
    void Run();

    Mat34<double> qInitMit;
    double kneeRatio = 1.0;

private:
    void InitRecv();
    void InitSend();

    ros::NodeHandle _nm;
    ros::Subscriber _servo_sub[16], _imu_sub, _model_sub;
    ros::Publisher _servo_pub[16];
    std::string _robot_name{"qr"};
    msg::qr::motor_ret motorData_;
    msg::wheel::motor_ret motorWheelRet_;
    Vec3<double> quat2RPY(Vec4<double> q);
    // Callback functions for ROS
    void SetJointCmd(const msg::qr::motor_cmd& msg);
    void SetJointCmd(const msg::wheel::motor_cmd& msg);

    void imuCallback(const sensor_msgs::Imu& msg);
    void FRhipCallback(const robot_msgs::MotorState& msg);
    void FRthighCallback(const robot_msgs::MotorState& msg);
    void FRcalfCallback(const robot_msgs::MotorState& msg);

    void FLhipCallback(const robot_msgs::MotorState& msg);
    void FLthighCallback(const robot_msgs::MotorState& msg);
    void FLcalfCallback(const robot_msgs::MotorState& msg);

    void RRhipCallback(const robot_msgs::MotorState& msg);
    void RRthighCallback(const robot_msgs::MotorState& msg);
    void RRcalfCallback(const robot_msgs::MotorState& msg);

    void RLhipCallback(const robot_msgs::MotorState& msg);
    void RLthighCallback(const robot_msgs::MotorState& msg);
    void RLcalfCallback(const robot_msgs::MotorState& msg);

    void FRwheelCallback(const robot_msgs::MotorState& msg);
    void FLwheelCallback(const robot_msgs::MotorState& msg);
    void RRwheelCallback(const robot_msgs::MotorState& msg);
    void RLwheelCallback(const robot_msgs::MotorState& msg);

    void ModelStateCallback(const gazebo_msgs::ModelStates::ConstPtr& msg);
};

#pragma once
#include <control_msgs/JointTrajectoryControllerState.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <cstdlib>

#include "baseline.hpp"

class SimArm
{
public:
    SimArm();
    void Run(std::string run_state);

private:
    void InitRecv();
    void InitSend();
    void PosFeedback(const control_msgs::JointTrajectoryControllerState& jointstate);
    void Send2Ros(std::string run_state);

    ros::NodeHandle _nm;
    ros::Subscriber _arm_sub;
    ros::Publisher _arm_pub;
    trajectory_msgs::JointTrajectory traj_;

    bool recieved_cmd_;

private:
    msg::arm_cmd cmd_;  // 等待下发的指令
    void SendMotorCmd();
};
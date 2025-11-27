#pragma once
#ifndef BASE_PATH_TRACKER__HPP
#define BASE_PATH_TRACKER__HPP

#include <iostream>
#include <Eigen/Eigen>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"


namespace navigation_core
{

class BasePathTracker {
public:
    BasePathTracker() = default;
    virtual std::optional<geometry_msgs::msg::Twist> compute_cmd_vel(const geometry_msgs::msg::Pose & current_pose, const geometry_msgs::msg::Twist & current_vel) = 0;
    virtual void set_path(const nav_msgs::msg::Path & path) = 0;
    virtual void configure(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> parent) = 0;
    ~BasePathTracker() = default;
private:

};

} // namespace 

#endif
#pragma once
#ifndef BASE_GOAL_CHECKER__HPP
#define BASE_GOAL_CHECKER__HPP

#include <iostream>
#include <Eigen/Eigen>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose.hpp"


namespace navigation_core
{

class BaseGoalChecker {

public:
    BaseGoalChecker() = default;
    virtual void configure(const rclcpp_lifecycle::LifecycleNode::SharedPtr parent) = 0;
    virtual bool is_goal_reached(const geometry_msgs::msg::Pose & current_pose) = 0;
    virtual void set_goal(const nav_msgs::msg::Path & path) = 0;
    virtual void reset() = 0;
    ~BaseGoalChecker() = default;
protected: 
    geometry_msgs::msg::Pose end_pose_;

};

} // namespace 

#endif
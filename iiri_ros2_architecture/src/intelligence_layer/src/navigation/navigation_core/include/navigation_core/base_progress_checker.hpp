#pragma once
#ifndef BASE_PROGRESS_CHECKER__HPP
#define BASE_PROGRESS_CHECKER__HPP

#include <Eigen/Eigen>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"


namespace navigation_core {

class BaseProgressChecker {
public: 
    BaseProgressChecker() = default;
    virtual void configure(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> parent) = 0;
    virtual bool check(const geometry_msgs::msg::Pose & current_pose) = 0;
    virtual void reset() = 0;
    ~BaseProgressChecker() = default;
};

}

#endif
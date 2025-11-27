#pragma once
#ifndef SIMPLE_PROGRESS_CHECKER__HPP
#define SIMPLE_PROGRESS_CHECKER__HPP

#include "navigation_core/base_goal_checker.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"


namespace path_tracker
{

class SimpleGoalChecker : public navigation_core::BaseGoalChecker {
public:
    SimpleGoalChecker() = default;
    void configure(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> parent) override;
    bool is_goal_reached(const geometry_msgs::msg::Pose & current_pose) override;
    void set_goal(const nav_msgs::msg::Path & path) override;
    void reset() override;
    ~SimpleGoalChecker();

private:
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
    double reach_goal_tolerance_;

};

} // namespace 

#endif




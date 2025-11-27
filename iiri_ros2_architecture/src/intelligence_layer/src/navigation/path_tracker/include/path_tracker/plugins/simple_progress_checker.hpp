#pragma once
#ifndef SIMPLE_PROGRESS_CHECKER__HPP
#define SIMPLE_PROGRESS_CHECKER__HPP

#include "navigation_core/base_progress_checker.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"


namespace path_tracker
{

class SimpleProgressChecker : public navigation_core::BaseProgressChecker {
public:
    SimpleProgressChecker() = default;
    void configure(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> parent) override;
    bool check(const geometry_msgs::msg::Pose & current_pose) override;
    void reset() override;
    ~SimpleProgressChecker();

private:
    geometry_msgs::msg::Pose baseline_pose_;
    rclcpp::Time baseline_time_;

    std::shared_ptr<rclcpp::Clock> clock_;
    rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
    rclcpp::Duration time_allowance_{0, 0};
    double move_radius_;
    bool flag_pose_set_;

    void reset_baseline(const geometry_msgs::msg::Pose & current_pose);
    bool is_move_enough(const geometry_msgs::msg::Pose & current_pose);
};

} // namespace 

#endif




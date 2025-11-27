#pragma once
#ifndef BASIC_PATH_TRACKER__HPP
#define BASIC_PATH_TRACKER__HPP

#include "navigation_core/base_path_tracker.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "extra/pose_utils.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
// debug:
#include "visualization_msgs/msg/marker.hpp"
// debug

namespace path_tracker
{

class BasicPathTracker : public navigation_core::BasePathTracker {
public:
    BasicPathTracker() = default;
    std::optional<geometry_msgs::msg::Twist> compute_cmd_vel(const geometry_msgs::msg::Pose & current_pose, const geometry_msgs::msg::Twist & current_vel) override;
    void set_path(const nav_msgs::msg::Path & path) override;
    void configure(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> parent) override;
    int get_closest_idx(const Eigen::Vector3d & current_position, std::pair<int, int> & segment_idx);
    std::optional<std::pair<int, int>> get_segment_idx(const Eigen::Vector3d & current_position);
    double get_closest_distance(const int & closest_idx, const geometry_msgs::msg::Pose & current_pose);
    int get_target_idx(const int & closest_idx, const double & track_forward_distance);
    double get_fit_length(const Eigen::MatrixXd & path_segment);
    void recv_closest_point(const std::shared_ptr<std_msgs::msg::Bool> collision_occur_msg);
    ~BasicPathTracker();

private:
    Eigen::MatrixXd path_;
    double achieve_goal_distance_;
    double track_forward_distance_;
    double fixed_velocity_; 
    double fixed_max_velocity_;
    PoseUtil pose_util_;
    double yaw_vel_fixed_;
    double tolerance_distance_;
    double tolerance_back_distance_;
    std::pair<int, int> last_segment_idx_;
    int time_init_ms_;
    int time_brake_ms_;
    double brake_vel_;
    std::chrono::steady_clock::time_point init_clock_;
    std::chrono::steady_clock::time_point brake_clock_;
    bool flag_collision_occur_, last_collision_occur_;
    std::mutex collision_detect_mutex_;

    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
    std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Bool>> collision_detect_sub_;
};

} // namespace 

#endif
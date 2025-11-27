#pragma once
#ifndef SIMPLE_PATH_TRACKER__HPP
#define SIMPLE_PATH_TRACKER__HPP

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

// Conditionally include Ceres based on compile-time flag
#ifdef USE_CERES_OPTIMIZATION
#include "ceres/ceres.h"
#endif

namespace path_tracker
{

#ifdef USE_CERES_OPTIMIZATION
// PathFitResidual struct for Ceres optimization
struct PathFitResidual {
    PathFitResidual(double x, double y, double theta, double weight_position, double weight_theta)
        : x_(x), y_(y), theta_(theta), weight_position_(weight_position), weight_theta_(weight_theta) {}

    template <typename T>
    bool operator()(const T* const angular_velocity, T* residual) const {
        // Position residual
        residual[0] = T(weight_position_) * (T(x_) - angular_velocity[0]);
        residual[1] = T(weight_position_) * (T(y_) - angular_velocity[1]);
        // Orientation residual
        residual[2] = T(weight_theta_) * (T(theta_) - angular_velocity[2]);
        return true;
    }

    static ceres::CostFunction* Create(double x, double y, double theta, double weight_position, double weight_theta) {
        return (new ceres::AutoDiffCostFunction<PathFitResidual, 3, 3>(
            new PathFitResidual(x, y, theta, weight_position, weight_theta)));
    }

    double x_, y_, theta_;
    double weight_position_, weight_theta_;
};
#endif

class SimplePathTracker : public navigation_core::BasePathTracker {
public:
    SimplePathTracker() = default;
    std::optional<geometry_msgs::msg::Twist> compute_cmd_vel(const geometry_msgs::msg::Pose & current_pose, const geometry_msgs::msg::Twist & current_vel) override;
    void set_path(const nav_msgs::msg::Path & path) override;
    void configure(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> parent) override;
    int get_closest_idx(const Eigen::Vector3d & current_position, std::pair<int, int> & segment_idx);
    std::optional<std::pair<int, int>> get_segment_idx(const Eigen::Vector3d & current_position);
    double get_closest_distance(const int & closest_idx, const geometry_msgs::msg::Pose & current_pose);
    int get_target_idx(const int & closest_idx, const double & track_forward_distance);
    double get_fit_length(const Eigen::MatrixXd & path_segment);
    double get_target_theta(const int & target_idx, const Eigen::Matrix4d & current_frame);
    void recv_closest_point(const std::shared_ptr<std_msgs::msg::Bool> collision_occur_msg);
    ~SimplePathTracker();

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
    double weight_position_;
    double weight_theta_;
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




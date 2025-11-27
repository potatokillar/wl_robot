#include "path_tracker/plugins/simple_progress_checker.hpp"

namespace path_tracker {

void SimpleProgressChecker::configure(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> parent) {
    node_ = parent;

    RCLCPP_INFO(node_->get_logger(), "SimpleProgressChecker configuration...");
    
    clock_ = node_->get_clock();
    flag_pose_set_ = false;

    node_->declare_parameter<double>("path_tracker/SimpleProgressChecker.time_allowance", 3.0);
    node_->declare_parameter<double>("path_tracker/SimpleProgressChecker.move_radius", 0.3);

    double time_allowance_d = node_->get_parameter("path_tracker/SimpleProgressChecker.time_allowance").as_double();
    move_radius_ = node_->get_parameter("path_tracker/SimpleProgressChecker.move_radius").as_double();  

    time_allowance_ = rclcpp::Duration::from_seconds(time_allowance_d);
}

bool SimpleProgressChecker::check(const geometry_msgs::msg::Pose & current_pose) {

    if (!flag_pose_set_ || is_move_enough(current_pose)) {
        reset_baseline(current_pose);
        return true;
    }
    return !((clock_->now() - baseline_time_) > time_allowance_);
}

void SimpleProgressChecker::reset_baseline(const geometry_msgs::msg::Pose & current_pose) { 
    baseline_pose_ = current_pose;
    baseline_time_ = clock_->now();
    flag_pose_set_ = true;
}

bool SimpleProgressChecker::is_move_enough(const geometry_msgs::msg::Pose & current_pose) {
    return std::hypot(
        current_pose.position.x - baseline_pose_.position.x,
        current_pose.position.y - baseline_pose_.position.y,
        current_pose.position.z - baseline_pose_.position.z
    ) > move_radius_;
}

void SimpleProgressChecker::reset() {
    flag_pose_set_ = false;
}

SimpleProgressChecker::~SimpleProgressChecker() {
    RCLCPP_WARN(node_->get_logger(), "SimpleProgressChecker deconstructing...");
    node_.reset();
    clock_.reset();
}

}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(path_tracker::SimpleProgressChecker, navigation_core::BaseProgressChecker)
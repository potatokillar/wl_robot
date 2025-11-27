#include "path_tracker/plugins/simple_goal_checker.hpp"

namespace path_tracker {

void SimpleGoalChecker::configure(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> parent) {
    node_ = parent;

    RCLCPP_INFO(node_->get_logger(), "SimpleGoalChecker configuration...");

    node_->declare_parameter<double>("path_tracker/SimpleGoalChecker.reach_goal_tolerance", 0.4);
    reach_goal_tolerance_ = node_->get_parameter("path_tracker/SimpleGoalChecker.reach_goal_tolerance").as_double();
}

bool SimpleGoalChecker::is_goal_reached(const geometry_msgs::msg::Pose & current_pose) {

    double distance_to_goal = std::hypot(
        current_pose.position.x - end_pose_.position.x,
        current_pose.position.y - end_pose_.position.y,
        current_pose.position.z - end_pose_.position.z
    );
    
    return distance_to_goal < reach_goal_tolerance_;
}

void SimpleGoalChecker::set_goal(const nav_msgs::msg::Path & path) { 
    end_pose_ = path.poses.back().pose;
};

void SimpleGoalChecker::reset() {}

SimpleGoalChecker::~SimpleGoalChecker() {
    RCLCPP_WARN(node_->get_logger(), "SimpleGoalChecker deconstructing...");
    node_.reset();
}

}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(path_tracker::SimpleGoalChecker, navigation_core::BaseGoalChecker)
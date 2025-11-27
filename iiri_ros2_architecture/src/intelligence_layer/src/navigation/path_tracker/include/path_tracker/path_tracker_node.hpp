#pragma once
#ifndef PATH_TRACKER__PATH_TRACKER_NODE__HPP
#define PATH_TRACKER__PATH_TRACKER_NODE__HPP

#include "navigation_core/base_path_tracker.hpp"
#include "navigation_core/base_progress_checker.hpp"
#include "navigation_core/base_goal_checker.hpp"
#include "extra/resource_tool.hpp"
#include "extra/pose_utils.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "pluginlib/class_loader.hpp"
#include <Eigen/Eigen>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <rclcpp_action/rclcpp_action.hpp>


namespace path_tracker {

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using PathTrackAction = nav2_msgs::action::FollowPath;
using PathTrackGoalHandle = rclcpp_action::ServerGoalHandle<PathTrackAction>;

class PathTrackerNode : public rclcpp_lifecycle::LifecycleNode {

public:

    PathTrackerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~PathTrackerNode();

protected:
    CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;

    void publish_zero_velocity();

private:    
    geometry_msgs::msg::Twist last_cmd_vel_;
    std::future<void> execute_future_;
    std::string current_path_tracker_;
    std::string current_goal_checker_;
    nav_msgs::msg::Path current_path_;
    std::string default_path_tracker_;
    std::string default_goal_checker_;
    double control_frequency_;
    rclcpp::Duration transform_tolerance_{0, 0};
    std::string global_frame_id_;
    std::string robot_frame_id_;
    std::string sensor_frame_id_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>> cmd_vel_pub_;

    /* tool instance */
    PoseUtil pose_utils_;

    /* plugins */
    std::shared_ptr<pluginlib::ClassLoader<navigation_core::BasePathTracker>> path_tracker_loader_;
    std::shared_ptr<pluginlib::ClassLoader<navigation_core::BaseProgressChecker>> progress_checker_loader_;
    std::shared_ptr<pluginlib::ClassLoader<navigation_core::BaseGoalChecker>> goal_checker_loader_;
    std::unordered_map<std::string, std::shared_ptr<navigation_core::BasePathTracker>> path_trackers_;
    std::unordered_map<std::string, std::shared_ptr<navigation_core::BaseGoalChecker>> goal_checkers_;
    std::shared_ptr<navigation_core::BaseProgressChecker> progress_checker_;


    /* action server relavant */
    std::shared_ptr<rclcpp_action::Server<PathTrackAction>> path_track_action_server_;
    std::shared_ptr<rclcpp::CallbackGroup> path_track_action_callback_group_;
    std::shared_ptr<PathTrackGoalHandle> current_handle_;
    std::shared_ptr<PathTrackGoalHandle> pending_handle_;

    /* thread */
    mutable std::recursive_mutex update_mutex_;

    /* flag */
    bool flag_active_;
    bool flag_stop_execute_;

    /* action server callback */
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const PathTrackAction::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<PathTrackGoalHandle> goal_handle);
    void handle_accept(const std::shared_ptr<PathTrackGoalHandle> goal_handle);

    /* main cycle */
    void work();

    /* wrapped utility */
    bool is_active_handle(const std::shared_ptr<PathTrackGoalHandle> & handle);
    void terminate_handle(std::shared_ptr<PathTrackGoalHandle> & handle);
    std::optional<geometry_msgs::msg::TransformStamped> get_transform(const std::string & input_frame, const std::string & target_frame, const rclcpp::Duration & timeout_tolerance);

    /* wrapped module function */
    bool update_handle();
    bool final_check_pending_handle();

};

}


#endif

#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/empty.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "extra/resource_tool.hpp"
#include <iostream>
#include <filesystem>
#include <fstream>
#include <sstream>


using namespace std;
using namespace std::placeholders;
using namespace std::chrono_literals;

namespace navigation_test {

using FollowPath = nav2_msgs::action::FollowPath;
using FollowPathGoalHandle = rclcpp_action::ClientGoalHandle<FollowPath>;

class SimplePathTrackerApi : public rclcpp::Node {
public:
    SimplePathTrackerApi(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()) : rclcpp::Node("simple_path_tracker_api_node", options) {
        send_goal_sub_ = this->create_subscription<std_msgs::msg::String>("send_track_path_goal", 10, 
            std::bind(&SimplePathTrackerApi::send_track_path_goal, this, _1));
        cancel_goal_sub_ = this->create_subscription<std_msgs::msg::Empty>("cancel_track_path_goal", 10, 
            std::bind(&SimplePathTrackerApi::cancel_track_path_goal, this, _1));
        path_track_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        path_tracker_action_client_ = rclcpp_action::create_client<FollowPath>(this, "track_path", path_track_callback_group_);
    }

private:
    std::shared_ptr<rclcpp_action::Client<FollowPath>> path_tracker_action_client_;
    std::shared_ptr<FollowPathGoalHandle> current_goal_handle_;
    std::shared_ptr<rclcpp::Subscription<std_msgs::msg::String>> send_goal_sub_;
    std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Empty>> cancel_goal_sub_;
    std::shared_ptr<rclcpp::CallbackGroup> path_track_callback_group_;
    std::mutex current_handle_mutex_;

    void send_track_path_goal(const std::shared_ptr<std_msgs::msg::String> path_msg) {
            
        if(!path_tracker_action_client_->wait_for_action_server(std::chrono::seconds(1))) {
            RCLCPP_ERROR(this->get_logger(), "path_tracker_action_server is not online!");
            return;
        }

        auto goal = FollowPath::Goal();
        ResourceTool resource_tool;
        std::string file_path = resource_tool.get_file_path("traj", path_msg->data);

        // debug:
        RCLCPP_WARN(this->get_logger(), "path_msg->data: %s", path_msg->data.c_str());
        // debug
        
        if (file_path.empty()) {
            RCLCPP_ERROR(this->get_logger(), "file specified doesn't exist !");
            return;
        }
        std::ifstream ifs(file_path);
        if (!ifs.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "file open failed !");
            return;
        }
        std::string line;
        std::getline(ifs, line);
        while (std::getline(ifs, line)) {
            std::string value;
            std::istringstream iss(line);
            geometry_msgs::msg::PoseStamped pose_stamped;
            std::getline(iss, value, ',');
            pose_stamped.header.frame_id = value;
            std::getline(iss, value, ',');
            pose_stamped.header.stamp.sec = std::stoi(value);
            std::getline(iss, value, ',');
            pose_stamped.header.stamp.nanosec = std::stoi(value);
            std::getline(iss, value, ',');
            pose_stamped.pose.position.x = std::stod(value);
            std::getline(iss, value, ',');
            pose_stamped.pose.position.y = std::stod(value);
            std::getline(iss, value, ',');
            pose_stamped.pose.position.z = std::stod(value);
            std::getline(iss, value, ',');
            pose_stamped.pose.orientation.x = std::stod(value);
            std::getline(iss, value, ',');
            pose_stamped.pose.orientation.y = std::stod(value);
            std::getline(iss, value, ',');
            pose_stamped.pose.orientation.z = std::stod(value);
            std::getline(iss, value, ',');
            pose_stamped.pose.orientation.w = std::stod(value);
            goal.path.poses.push_back(pose_stamped);
        }
        auto send_goal_options = rclcpp_action::Client<FollowPath>::SendGoalOptions();
        send_goal_options.goal_response_callback = 
            std::bind(&SimplePathTrackerApi::goal_response_callback, this, _1);
        send_goal_options.feedback_callback = 
            std::bind(&SimplePathTrackerApi::feedback_callback, this, _1, _2);
        send_goal_options.result_callback = 
            std::bind(&SimplePathTrackerApi::result_callback, this, _1);
        auto future = path_tracker_action_client_->async_send_goal(goal, send_goal_options);
        if (future.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
            RCLCPP_ERROR(this->get_logger(), "Send_goal request wait for response timeout ! ");
        }
    }

    void cancel_track_path_goal(const std::shared_ptr<std_msgs::msg::Empty> ) {
        std::lock_guard<std::mutex> lock(current_handle_mutex_);

        if (!current_goal_handle_) {
            RCLCPP_ERROR(this->get_logger(), "Current goal handle is NULL !");
            return;
        }

        if (current_goal_handle_->get_status() != action_msgs::msg::GoalStatus::STATUS_EXECUTING && 
            current_goal_handle_->get_status() != action_msgs::msg::GoalStatus::STATUS_ACCEPTED) {
            RCLCPP_ERROR(this->get_logger(), "Current goal handle is not active");
            return;
        }

        if(!path_tracker_action_client_->wait_for_action_server(std::chrono::seconds(1))) {
            RCLCPP_ERROR(this->get_logger(), "path_tracker_action_server is not online!");
            return;
        }

        auto cancel_future = path_tracker_action_client_->async_cancel_goal(current_goal_handle_);
        if (cancel_future.wait_for(std::chrono::seconds(1)) != std::future_status::ready) {
            RCLCPP_WARN(this->get_logger(), "Cancel request wait for response timeout !");
            return;
        }
        if (cancel_future.valid()) {
            RCLCPP_INFO(get_logger(), "Cancel request recieved");
        }
        return;
    }

    void goal_response_callback(const FollowPathGoalHandle::SharedPtr & goal_handle) {
        std::lock_guard<std::mutex> lock(current_handle_mutex_);
        if (!goal_handle) {
            RCLCPP_WARN(this->get_logger(), "path track request was rejected by service !");
            return;
        }
        current_goal_handle_ = goal_handle;
        RCLCPP_INFO(this->get_logger(), "path track request was accept by service !");
    }

    void feedback_callback(FollowPathGoalHandle::SharedPtr, const std::shared_ptr<const FollowPath::Feedback> feedback) {
        RCLCPP_INFO(this->get_logger(), "distance_to_goal: %f", feedback->distance_to_goal);
        RCLCPP_INFO(this->get_logger(), "distance_to_goal: %f", feedback->speed);
        return;
    };

    void result_callback(const FollowPathGoalHandle::WrappedResult & result) {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED: {
                RCLCPP_INFO(this->get_logger(), "track path action task have reach the goal");
                return;
            } 
            case rclcpp_action::ResultCode::ABORTED: {
                RCLCPP_WARN(this->get_logger(), "track path action task have been aborted");
                return;
            }
            case rclcpp_action::ResultCode::CANCELED: {
                RCLCPP_INFO(this->get_logger(), "track path action task have been canceled");
                return;
            }
            default: {
                RCLCPP_ERROR(this->get_logger(), "Unknown result code !");
                return;
            }
        }
    }

};

}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor exec;
    auto node = std::make_shared<navigation_test::SimplePathTrackerApi>();
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
    
#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker.hpp"
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

class PathTrackerTest : public rclcpp::Node {
public:
    PathTrackerTest(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()) : rclcpp::Node("path_tracker_test_node", options) {
        this->declare_parameter("test_path_file", "traj-2025-06-03-22:15:58.csv");
        test_path_file_ = this->get_parameter("test_path_file").as_string();
        send_goal_server_ = this->create_service<std_srvs::srv::Trigger>("send_track_path_goal", 
            std::bind(&PathTrackerTest::send_track_path_goal, this, _1, _2));
        cancel_goal_server_ = this->create_service<std_srvs::srv::Trigger>("cancel_track_path_goal", 
            std::bind(&PathTrackerTest::cancel_track_path_goal, this, _1, _2));
        cancel_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        path_tracker_action_client_ = rclcpp_action::create_client<FollowPath>(this, "track_path", cancel_callback_group_);
        tts_content_pub_ = this->create_publisher<std_msgs::msg::String>("/tts/content", 10);
    }

private:
    std::shared_ptr<rclcpp_action::Client<FollowPath>> path_tracker_action_client_;
    std::shared_ptr<FollowPathGoalHandle> current_goal_handle_;
    std::string test_path_file_;
    std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> send_goal_server_;
    std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> cancel_goal_server_;
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::String>> tts_content_pub_;
    std::shared_ptr<rclcpp::CallbackGroup> cancel_callback_group_;
    std::mutex current_handle_mutex_;

    void send_track_path_goal(const std::shared_ptr<std_srvs::srv::Trigger::Request> &, 
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
            
        if(!path_tracker_action_client_->wait_for_action_server(std::chrono::seconds(1))) {
            RCLCPP_ERROR(this->get_logger(), "path_tracker_action_server is not online!");
            return;
        }

        auto goal = FollowPath::Goal();
        ResourceTool resource_tool;
        std::string file_path = resource_tool.get_file_path("traj", test_path_file_);
        if (file_path.empty()) {
            RCLCPP_ERROR(this->get_logger(), "file specified doesn't exist !");
            response->success = false;
            return;
        }
        std::ifstream ifs(file_path);
        if (!ifs.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "file open failed !");
            response->success = false;
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
            std::bind(&PathTrackerTest::goal_response_callback, this, _1);
        send_goal_options.feedback_callback = 
            std::bind(&PathTrackerTest::feedback_callback, this, _1, _2);
        send_goal_options.result_callback = 
            std::bind(&PathTrackerTest::result_callback, this, _1);
        path_tracker_action_client_->async_send_goal(goal, send_goal_options);
        
        response->success = true;
    }

    void cancel_track_path_goal(const std::shared_ptr<std_srvs::srv::Trigger::Request> &, std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        std::lock_guard<std::mutex> lock(current_handle_mutex_);

        if (!current_goal_handle_) {
            RCLCPP_ERROR(this->get_logger(), "Current goal handle is NULL !");
            response->message = "Current goal handle is NULL!";
            std_msgs::msg::String tty_msg;
            tty_msg.data = "暂停失败，任务未执行";
            tts_content_pub_->publish(tty_msg);
            response->success = false;
            return;
        }

        if (current_goal_handle_->get_status() != action_msgs::msg::GoalStatus::STATUS_EXECUTING && 
            current_goal_handle_->get_status() != action_msgs::msg::GoalStatus::STATUS_ACCEPTED) {
            RCLCPP_ERROR(this->get_logger(), "Current goal handle is not active");
            response->message = "Current goal handle is not active!";
            std_msgs::msg::String tty_msg;
            tty_msg.data = "暂停失败，任务未执行";
            tts_content_pub_->publish(tty_msg);
            response->success = false;
            return;
        }

        if(!path_tracker_action_client_->wait_for_action_server(std::chrono::seconds(1))) {
            RCLCPP_ERROR(this->get_logger(), "path_tracker_action_server is not online!");
            response->message = "path_tracker_action_server is not online!";
            std_msgs::msg::String tty_msg;
            tty_msg.data = "暂停失败，服务端未上线";
            tts_content_pub_->publish(tty_msg);
            response->success = false;
            return;
        }

        auto cancel_future = path_tracker_action_client_->async_cancel_goal(current_goal_handle_);
        while(cancel_future.wait_for(std::chrono::seconds(1)) != std::future_status::ready) {
            RCLCPP_WARN(this->get_logger(), "Cancel request wait for response !");
        }
        if (cancel_future.valid()) {
            RCLCPP_INFO(get_logger(), "Cancel request recieved");
        } 
        response->message = "Cancel request recieved";
        std_msgs::msg::String tty_msg;
        tty_msg.data = "暂停成功";
        tts_content_pub_->publish(tty_msg);
        response->success = true;
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
                std_msgs::msg::String tty_msg;
                tty_msg.data = "任务已完成";
                tts_content_pub_->publish(tty_msg);
                return;
            } 
            case rclcpp_action::ResultCode::ABORTED: {
                RCLCPP_WARN(this->get_logger(), "track path action task have been aborted");
                std_msgs::msg::String tty_msg;
                tty_msg.data = "任务已终止";
                tts_content_pub_->publish(tty_msg);
                return;
            }
            case rclcpp_action::ResultCode::CANCELED: {
                RCLCPP_INFO(this->get_logger(), "track path action task have been canceled");
                std_msgs::msg::String tty_msg;
                tty_msg.data = "任务已取消";
                tts_content_pub_->publish(tty_msg);
                return;
            }
            default: {
                RCLCPP_ERROR(this->get_logger(), "Unknown result code !");
                std_msgs::msg::String tty_msg;
                tty_msg.data = "未知错误, 路径跟踪失败";
                tts_content_pub_->publish(tty_msg);
                return;
            }
        }
    }

};

}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor exec;
    auto node = std::make_shared<navigation_test::PathTrackerTest>();
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
    
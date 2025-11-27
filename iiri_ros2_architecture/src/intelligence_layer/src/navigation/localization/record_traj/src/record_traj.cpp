#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "extra/resource_tool.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "ament_index_cpp/get_package_prefix.hpp"
#include "Eigen/Geometry"
#include "Eigen/Eigen"
#include <filesystem>
#include <iostream>
#include <sstream>
#include <fstream>
#include <deque>


using namespace std::placeholders;

class RecordTraj : public rclcpp::Node {

public:

    RecordTraj(const std::string node_name) : rclcpp::Node(node_name) {
        init_param();
        record_traj_serv_ = this->create_service<std_srvs::srv::Trigger>("start_record_traj", std::bind(&RecordTraj::record_traj_callback, this, _1, _2));
        save_traj_serv_ = this->create_service<std_srvs::srv::Trigger>("save_traj", std::bind(&RecordTraj::save_traj_callback, this, _1, _2));
        traj_pub_ = this->create_publisher<nav_msgs::msg::Path>("traj", 10);
        stop_record_traj_serv_ = this->create_service<std_srvs::srv::Trigger>("stop_record_traj", std::bind(&RecordTraj::stop_record_traj_callback, this, _1, _2));

    }
      
    void init_param() {
        path_.poses.resize(0);
        flag_recording = false;
        last_record_pos_.resize(0);

        // 初始化参数
        this->declare_parameter<double>("record_max_distance", 0.15);
        this->declare_parameter<double>("record_min_distance", 0.1);
        this->declare_parameter<double>("record_max_angle", 100.0);
        this->declare_parameter<std::string>("traj_parent_directory", "traj");
        this->declare_parameter<std::string>("traj_file_name", "traj.csv");

        record_max_distance_ = this->get_parameter("record_max_distance").as_double();
        record_min_distance_ = this->get_parameter("record_min_distance").as_double();
        record_max_angle_ = this->get_parameter("record_max_angle").as_double();
        traj_parent_directory_ = this->get_parameter("traj_parent_directory").as_string();
        traj_file_name_ = this->get_parameter("traj_file_name").as_string();
    }
    
    ~RecordTraj() {}

private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr record_traj_serv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_record_traj_serv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_traj_serv_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr traj_pub_;
    nav_msgs::msg::Path path_;
    std::deque<Eigen::Vector3d> last_record_pos_;
    double record_max_distance_;
    double record_min_distance_;
    double record_max_angle_;
    std::string traj_parent_directory_;
    std::string traj_file_name_;
    bool flag_recording;
    ResourceTool resource_tool;

    nav_msgs::msg::Path read_traj_from_csv(const std::string & traj_file_path) {
        nav_msgs::msg::Path traj;
        traj.header.stamp = this->now();

        std::ifstream file(traj_file_path);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open file: %s", traj_file_path.c_str());
            return traj;
        }

        std::string line;
        std::getline(file, line);
        while (std::getline(file, line)) {
            std::string value;
            geometry_msgs::msg::PoseStamped pose;
            std::istringstream line_stream(line);
            std::getline(line_stream, value, ',');
            pose.header.frame_id = value;
            std::getline(line_stream, value, ',');
            pose.header.stamp.sec = std::stoi(value);
            std::getline(line_stream, value, ',');
            pose.header.stamp.nanosec = std::stoi(value);
            std::getline(line_stream, value, ',');
            pose.pose.position.x = std::stod(value);
            std::getline(line_stream, value, ',');
            pose.pose.position.y = std::stod(value);
            std::getline(line_stream, value, ',');
            pose.pose.position.z = std::stod(value);
            std::getline(line_stream, value, ',');
            pose.pose.orientation.x = std::stod(value);
            std::getline(line_stream, value, ',');
            pose.pose.orientation.y = std::stod(value);
            std::getline(line_stream, value, ',');
            pose.pose.orientation.z = std::stod(value);
            std::getline(line_stream, value);
            pose.pose.orientation.w = std::stod(value);
            traj.header.frame_id = pose.header.frame_id;
            traj.poses.emplace_back(pose);
        }
        file.close();
        return traj;
    }

    void record_traj_point_callback(const std::shared_ptr<nav_msgs::msg::Odometry> odom_msg) {
        Eigen::Vector3d cur_pos(
            odom_msg->pose.pose.position.x,
            odom_msg->pose.pose.position.y,
            odom_msg->pose.pose.position.z
        );
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.pose.position.x = odom_msg->pose.pose.position.x;
        pose_stamped.pose.position.y = odom_msg->pose.pose.position.y;
        pose_stamped.pose.position.z = odom_msg->pose.pose.position.z;
        pose_stamped.pose.orientation.x = odom_msg->pose.pose.orientation.x;
        pose_stamped.pose.orientation.y = odom_msg->pose.pose.orientation.y;
        pose_stamped.pose.orientation.z = odom_msg->pose.pose.orientation.z;
        pose_stamped.pose.orientation.w = odom_msg->pose.pose.orientation.w;
        pose_stamped.header.frame_id = odom_msg->header.frame_id;
        pose_stamped.header.stamp.sec = odom_msg->header.stamp.sec;
        pose_stamped.header.stamp.nanosec = odom_msg->header.stamp.nanosec;

        if (path_.poses.empty()) {
            path_.header.stamp = this->now();
            path_.header.frame_id = pose_stamped.header.frame_id;
            path_.poses.emplace_back(pose_stamped);
            last_record_pos_.emplace_front(cur_pos);
            traj_pub_->publish(path_);
            return;
        }
        double distance = (cur_pos - last_record_pos_.front()).norm();
        double angle = 0.0;
        if (last_record_pos_.size() > 1) {
            Eigen::Vector3d vec_last = (last_record_pos_.front() - last_record_pos_.back()).normalized();
            Eigen::Vector3d vec_cur = (cur_pos - last_record_pos_.front()).normalized();
            angle = acos(vec_last.dot(vec_cur)) / M_PI * 180.0; 
        }
        if (distance >= record_max_distance_ || (distance > record_min_distance_ && angle < record_max_angle_)) {
            path_.poses.emplace_back(pose_stamped);
            last_record_pos_.emplace_front(cur_pos);
            if (last_record_pos_.size() > 2) {
                last_record_pos_.pop_back();
            }
            traj_pub_->publish(path_);
        }
    }

    void record_traj_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        (void)req;
        if (flag_recording) {
            RCLCPP_WARN(this->get_logger(), "Is recording, don't recall record again !");
            res->success = false;
            res->message = "Is recording. Call record failed!";
            return;
        }
        flag_recording = true;
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&RecordTraj::record_traj_point_callback, this, _1));
        res->success = true;
        res->message = "Start record path!";
        RCLCPP_INFO(this->get_logger(), "Start record path!");
    }

    void stop_record_traj_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        (void)req;
        flag_recording = false;
        odom_sub_.reset();
        path_.poses.resize(0);
        last_record_pos_.resize(0);
        res->success = true;
        res->message = "Stop record path !";
        RCLCPP_INFO(this->get_logger(), "Stop record path !");
    }

    void save_traj_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        (void)req;
        if (path_.poses.empty()) {
            RCLCPP_WARN(this->get_logger(), "Path is empty, please start record path before save path.");
            res->success = false;
            res->message = "Path is empty. Record path failed !";
            return;
        }

        std::string file_path = resource_tool.create_file(traj_parent_directory_, traj_file_name_);

        std::ofstream file(file_path);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "File open failed!");
            res->success = false;
            res->message = "File open failed !";
            return;
        }
        file << "frame_id,sec,nanosec,pos_x,pos_y,pos_z,orient_x,orient_y,orient_z,orient_w\n";
        for (const auto& pose : path_.poses) {
            file<< pose.header.frame_id << ","
                << pose.header.stamp.sec << ","
                << pose.header.stamp.nanosec << ","
                << pose.pose.position.x << ","
                << pose.pose.position.y << ","
                << pose.pose.position.z << ","
                << pose.pose.orientation.x << ","
                << pose.pose.orientation.y << ","
                << pose.pose.orientation.z  << ","
                << pose.pose.orientation.w << "\n";
        }
        file.close();   // 关闭文件流，否则数据存在缓冲区中，后续复制操作只能复制文件，不能将内容数据进行复制

        RCLCPP_INFO(this->get_logger(), "Path saved to: %s", file_path.c_str());
        res->success = true;
        res->message = "Save path success !";
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RecordTraj>("record_traj_node");
    RCLCPP_INFO(node->get_logger(), "Hello record_traj_node!");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
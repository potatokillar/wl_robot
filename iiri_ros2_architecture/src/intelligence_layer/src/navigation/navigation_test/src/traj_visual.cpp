#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "extra/resource_tool.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
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

class TrajVisual : public rclcpp::Node {
public:
    TrajVisual(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()) : rclcpp::Node("traj_visual_node", options) {
        this->declare_parameter("traj_file_name", "traj-2025-06-17-10:05:27.csv");
        this->declare_parameter("pcl_file_name", "horizon_map.pcd");
        traj_file_name_ = this->get_parameter("traj_file_name").as_string(); 
        pcl_file_name_ = this->get_parameter("pcl_file_name").as_string(); 
        traj_visual_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("traj_visual", 10);
        global_map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("global_map", 10);
        traj_visual_timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&TrajVisual::traj_visual_timer_callback, this));
        global_map_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    }

private:
    ResourceTool resource_tool;
    std::string traj_file_name_;
    std::string pcl_file_name_;
    std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::Marker>> traj_visual_pub_;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> global_map_pub_;
    std::shared_ptr<rclcpp::TimerBase> traj_visual_timer_;
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> global_map_;

    void traj_visual_timer_callback() {
        // 发布点云地图
        std::string pcl_file_path = resource_tool.get_file_path("map", pcl_file_name_);
        if (pcl_file_path.empty()) {
            RCLCPP_ERROR(this->get_logger(), "pcl_file_name is not correct !");
            return;
        }
        pcl::io::loadPCDFile(pcl_file_path, *global_map_); // 如果使用XYZI会报出错误没有设置intensity属性

        sensor_msgs::msg::PointCloud2 global_map_ros;
        pcl::toROSMsg(*global_map_, global_map_ros);
        global_map_ros.header.frame_id = "map";
        
        global_map_pub_->publish(global_map_ros);

        std::string file_path = resource_tool.get_file_path("traj", traj_file_name_);
        if (file_path.empty()) {
            RCLCPP_ERROR(this->get_logger(), "file specified doesn't exist !");
            return;
        }
        std::ifstream ifs(file_path);
        if (!ifs.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "file open failed !");
            return;
        }

        visualization_msgs::msg::Marker traj_msg;
        traj_msg.action = visualization_msgs::msg::Marker::ADD;
        traj_msg.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        traj_msg.color.r = 0.3;
        traj_msg.color.g = 0.6;
        traj_msg.color.b = 0.1;
        traj_msg.color.a = 1.0;
        traj_msg.id = 0;
        traj_msg.header.frame_id = "map";
        traj_msg.header.stamp = this->get_clock()->now();
        traj_msg.scale.x = 0.08;
        traj_msg.scale.y = 0.08;
        traj_msg.scale.z = 0.08;
        traj_msg.pose.orientation.x = 0.0;
        traj_msg.pose.orientation.y = 0.0;
        traj_msg.pose.orientation.z = 0.0;
        traj_msg.pose.orientation.w = 1.0;
        traj_msg.pose.position.x = 0.0;
        traj_msg.pose.position.y = 0.0;
        traj_msg.pose.position.z = 0.0;
        
        std::string line;
        std::getline(ifs, line);
        while (std::getline(ifs, line)) {
            geometry_msgs::msg::Point traj_point;
            std::string value;
            std::istringstream iss(line);
            std::getline(iss, value, ',');
            std::getline(iss, value, ',');
            std::getline(iss, value, ',');
            std::getline(iss, value, ',');
            traj_point.x = std::stod(value);
            std::getline(iss, value, ',');
            traj_point.y = std::stod(value);
            std::getline(iss, value, ',');
            traj_point.z = std::stod(value);
            std::getline(iss, value, ',');
            std::getline(iss, value, ',');
            std::getline(iss, value, ',');
            std::getline(iss, value, ',');
            traj_msg.points.push_back(traj_point);
        }
        traj_visual_pub_->publish(traj_msg);
    }
};

}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<navigation_test::TrajVisual>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
    
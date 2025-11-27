#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include "extra/resource_tool.hpp"
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>
#include <filesystem>
#include <iostream>
#include <sstream>


class MakeMapHorizontal : public rclcpp::Node {
public:

    MakeMapHorizontal(const std::string node_name) : rclcpp::Node(node_name) {

        record_point_time_ = 0;
        flag_recv_odom_ = false;
        this->declare_parameter<std::string>("map_file_parent_directory", "map");
        this->declare_parameter<std::string>("map_file_name", "horizon_map.pcd");
        this->declare_parameter<std::string>("global_map_frame_id", "map");

        map_file_parent_directory_ = this->get_parameter("map_file_parent_directory").as_string();
        map_file_name_ = this->get_parameter("map_file_name").as_string();
        global_map_frame_id_ = this->get_parameter("global_map_frame_id").as_string();
        
        odom_point_.setZero();
        point1_.setZero();
        point2_.setZero();
        point3_.setZero();
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/Odometry", 20, std::bind(&MakeMapHorizontal::odom_recv_callback, this, std::placeholders::_1));
        record_point_serv_ = this->create_service<std_srvs::srv::Trigger>("record_horizontal_point", std::bind(&MakeMapHorizontal::record_point_callback, this, std::placeholders::_1, std::placeholders::_2));
        map_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/Laser_map", 10, std::bind(&MakeMapHorizontal::map_recv_callback, this, std::placeholders::_1));
        map_save_serv_ = this->create_service<std_srvs::srv::Trigger>("save_horizontal_map", std::bind(&MakeMapHorizontal::map_save_callback, this, std::placeholders::_1, std::placeholders::_2));
    }

    ~MakeMapHorizontal() {}

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr map_save_serv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr record_point_serv_;
    int record_point_time_;            // 记录水平位置点的次数
    bool flag_recv_odom_;
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> global_map_;
    std::string map_file_name_;
    std::string map_file_parent_directory_;
    std::string global_map_frame_id_;
    
    Eigen::Vector3d odom_point_, point1_, point2_, point3_;

    void odom_recv_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg) {
        odom_point_.x() = odom_msg->pose.pose.position.x;
        odom_point_.y() = odom_msg->pose.pose.position.y;
        odom_point_.z() = odom_msg->pose.pose.position.z;
        flag_recv_odom_ = true;
    }

    void record_point_callback(std_srvs::srv::Trigger::Request::ConstSharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res) {
        (void)req;
        if (!flag_recv_odom_) {
            RCLCPP_WARN(this->get_logger(), "Odom point haven't recieved!");
            record_point_time_ = 0;
            res->success = false;
            res->message = "Record failed!";
            return;
        }
        if (record_point_time_ < 1) {
            point1_ = odom_point_;
            record_point_time_++;
            RCLCPP_WARN(this->get_logger(), "One point recorded!");
            res->success = true;
            res->message = "Record success!";
        } else if (record_point_time_ < 2) {
            point2_ = odom_point_;
            record_point_time_++;
            RCLCPP_WARN(this->get_logger(), "Two point recorded!");
            res->success = true;
            res->message = "Record success!";
        } else if (record_point_time_ < 3) {
            point3_ = odom_point_;
            record_point_time_++;
            RCLCPP_WARN(this->get_logger(), "Three point recorded!");
            res->success = true;
            res->message = "Record success!";
        } else {
            RCLCPP_WARN(this->get_logger(), "Three point have been recorded!");
            res->success = true;
            res->message = "Have recorded!";
        }
    }

    void map_save_callback(std_srvs::srv::Trigger::Request::ConstSharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res) {
        (void)req;
        if (!global_map_) {
            RCLCPP_WARN(this->get_logger(), "Global map haven't recieved !");
            res->success = false;
            res->message = "Map save failed!";
            return;
        }
        if (record_point_time_ < 3) {
            RCLCPP_WARN(this->get_logger(), "Three point haven't record completely!");
            res->success = false;
            res->message = "Map save failed!";
            return;
        }

        Eigen::Matrix4f transform_horizontal = get_transform();

        pcl::PointCloud<pcl::PointXYZ> global_map_horizontal;
        pcl::transformPointCloud(*global_map_, global_map_horizontal, transform_horizontal);
        global_map_horizontal.header.frame_id = global_map_frame_id_;

        ResourceTool resource_tool;
        std::string map_file_path = resource_tool.create_file(map_file_parent_directory_, map_file_name_);

        pcl::PCDWriter pcd_writer;
        pcd_writer.writeBinary(map_file_path, global_map_horizontal);
        RCLCPP_INFO(this->get_logger(), "Save map into: %s", map_file_path.c_str());
        res->success = true;
        res->message = "Map save success!";
    }

    void map_recv_callback(const sensor_msgs::msg::PointCloud2::SharedPtr map_msg) {
        auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        pcl::fromROSMsg(*map_msg, *cloud);

        if (cloud->empty()) {
            RCLCPP_WARN(this->get_logger(),"Recieved cloud map is empty !");
            return;
        }
        global_map_ = cloud;
    }

    Eigen::Matrix4f get_transform() {
        // 1. 首先根据三个点计算平面法向量
        Eigen::Vector3d vec13, vec12;
        vec12 = point2_ - point1_;
        vec13 = point3_ - point1_;
        Eigen::Vector3d norm_vec = vec13.cross(vec12);
        norm_vec = norm_vec.z() < 0 ? -norm_vec : norm_vec;
        norm_vec.normalize();

        // 2. 法向量与Z轴叉乘计算螺旋轴
        Eigen::Vector3d z_axis(0, 0, 1);
        Eigen::Vector3d axis = norm_vec.cross(z_axis).normalized();

        // 3. 利用反余弦函数来计算旋转角度
        double angle = acos(z_axis.dot(norm_vec));

        // 4. 通过螺旋轴以及旋转角度初始化Eigen旋转矩阵，通过eigen方法返回旋转矩阵
        Eigen::AngleAxisd rotation(angle, axis);
        Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
        transform.block<3, 3>(0, 0) = rotation.toRotationMatrix();
        return transform.cast<float>();
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MakeMapHorizontal>("horizon_map_node");
    RCLCPP_INFO(node->get_logger(), "Hello make_map_horizontal_node!");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
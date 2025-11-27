/*
 * @Author: 周健伟
 * @Date: 2025-04-29
 * @LastEditors: 周健伟
 * @LastEditTime: 2025-06-15
 * @Description:
 *
 * Copyright (c) 2022-2025 by 西湖大学智能产业研究院, All Rights Reserved.
 */
#include "rclcpp/rclcpp.hpp"
#include <Eigen/Eigen>
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/bool.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/filters/radius_outlier_removal.h>
#include <iostream>

/** 
 * @brief: 检测激光雷达坐标系下x轴方向一定扇形区域下的最近点位置
 */

class CollisionDetect : public rclcpp::Node {
public:
    CollisionDetect(std::string node_name) : rclcpp::Node(node_name) {
        RCLCPP_INFO(this->get_logger(), "Hello CollisionDetect node !");

        // 声明并获取俯仰角参数以及有效距离阈值参数;
        this->declare_parameter<float>("pitch_thresh", 7.0); // 检测最近点的俯仰角范围
        this->declare_parameter<float>("dist_thresh",  0.2);
        float pitch_thresh = this->get_parameter("pitch_thresh").as_double();
        sin_thresh_.first = -sin(pitch_thresh / 180.0 * M_PI);
        sin_thresh_.second = sin(pitch_thresh / 180.0 * M_PI);
        
        dist_thresh_ = this->get_parameter("dist_thresh").as_double();
        
        collision_detect_pub_ = this->create_publisher<std_msgs::msg::Bool>("collision_occur", 10);

        point_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("point_cloud_in", 10, 
            std::bind(&CollisionDetect::RecvPclCallback, this, std::placeholders::_1));
    }
private: 
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr collision_detect_pub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;

    std::pair<float, float> sin_thresh_;
    float dist_thresh_;

    void RecvPclCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_ptr(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*msg, *pcl_ptr);

        // 将无效的NaN点去除
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*pcl_ptr, *pcl_ptr, indices);

        if (pcl_ptr->empty()) {
            return;
        }

        // for (auto & point : *filted_pcl_ptr) {
        for (const auto & point : *pcl_ptr) {
            if (point.x < 1e-4 || point.x > dist_thresh_) {
                // Remove the points in the back region
                continue;
            }
            float distance = std::hypot(point.x, point.y, point.z);
            float sin_value = point.z / distance;
            if (sin_value < sin_thresh_.first || sin_value > sin_thresh_.second) {
                continue;
            };
            if (distance < dist_thresh_) {
                std_msgs::msg::Bool collision_occur;
                collision_occur.data = true;
                collision_detect_pub_->publish(collision_occur);
                return;
            };
        };
        std_msgs::msg::Bool collision_occur;
        collision_occur.data = false;
        collision_detect_pub_->publish(collision_occur);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CollisionDetect>("collision_detect_node");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
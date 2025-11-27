/*
 * @Author: 周健伟
 * @Date: 2025-04-29
 * @LastEditors: 周健伟
 * @LastEditTime: 2025-04-29
 * @Description:
 *
 * Copyright (c) 2022-2025 by 西湖大学智能产业研究院, All Rights Reserved.
 */
#pragma once
#ifndef POSE_UTIL__HPP
#define POSE_UTIL__HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <Eigen/Eigen>
#include <Eigen/Geometry>


class PoseUtil {
public:
    PoseUtil() = default;
    geometry_msgs::msg::Pose get_pose(const geometry_msgs::msg::Transform);
    geometry_msgs::msg::Pose get_pose(const geometry_msgs::msg::TransformStamped);
    Eigen::Matrix4d pose_to_matrix(const geometry_msgs::msg::Pose & pose);
    Eigen::Matrix4d pose_to_matrix(const geometry_msgs::msg::PoseStamped & pose);
    Eigen::Matrix4d transform_to_matrix(const geometry_msgs::msg::Transform & transform);
    Eigen::Matrix4d transform_to_matrix(const geometry_msgs::msg::TransformStamped & transform);
    ~PoseUtil() = default;
};

#endif

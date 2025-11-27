/*
 * @Author: 周健伟
 * @Date: 2025-04-29
 * @LastEditors: 周健伟
 * @LastEditTime: 2025-06-15
 * @Description:
 *
 * Copyright (c) 2022-2025 by 西湖大学智能产业研究院, All Rights Reserved.
 */
#include "extra/pose_utils.hpp"


geometry_msgs::msg::Pose PoseUtil::get_pose(const geometry_msgs::msg::Transform transform) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = transform.translation.x;
    pose.position.y = transform.translation.y;
    pose.position.z = transform.translation.z;
    pose.orientation = transform.rotation;
    return pose;
}
geometry_msgs::msg::Pose PoseUtil::get_pose(const geometry_msgs::msg::TransformStamped transform_stamped) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = transform_stamped.transform.translation.x;
    pose.position.y = transform_stamped.transform.translation.y;
    pose.position.z = transform_stamped.transform.translation.z;
    pose.orientation = transform_stamped.transform.rotation;
    return pose;
}
Eigen::Matrix4d PoseUtil::pose_to_matrix(const geometry_msgs::msg::Pose & pose) {
    Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
    matrix.block<3, 1>(0, 3) << pose.position.x, pose.position.y, pose.position.z;
    Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
    matrix.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
    return matrix; 
}
Eigen::Matrix4d PoseUtil::pose_to_matrix(const geometry_msgs::msg::PoseStamped & pose) {
    Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
    matrix.block<3, 1>(0, 3) << pose.pose.position.x, pose.pose.position.y, pose.pose.position.z;
    Eigen::Quaterniond q(pose.pose.orientation.w, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z);
    matrix.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
    return matrix; 
}
Eigen::Matrix4d PoseUtil::transform_to_matrix(const geometry_msgs::msg::Transform & transform) {
    Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
    matrix.block<3, 1>(0, 3) << transform.translation.x, transform.translation.y, transform.translation.z;
    Eigen::Quaterniond q(transform.rotation.w, transform.rotation.x, transform.rotation.y, transform.rotation.z);
    matrix.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
    return matrix; 
}
Eigen::Matrix4d PoseUtil::transform_to_matrix(const geometry_msgs::msg::TransformStamped & transform) {
    Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
    matrix.block<3, 1>(0, 3) << transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z;
    Eigen::Quaterniond q(transform.transform.rotation.w, transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z);
    matrix.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
    return matrix; 
}

#ifndef REALSENSE_CAMERA_HPP
#define REALSENSE_CAMERA_HPP
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <librealsense2/rs.hpp>

class RealsenseCamera : public rclcpp::Node
{
public:
    RealsenseCamera();
    void InitParam();
    void OpenCamera();


private:
    struct Config {
        int device_id;
        int frame_width;
        int frame_height;
        int fps;
        bool undistort;
        cv::Mat intrinsic;
        cv::Mat dist_coeff;
    };

    // 创建realsense管道
    rs2::pipeline _p;
    // 全局参数
    Config _config;
    cv_bridge::CvImage _cv2msg;
    rclcpp::TimerBase::SharedPtr _timer;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _img_pub;
    void PubImgCallback();
};

#endif

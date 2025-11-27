#ifndef USB_CAMERA__HPP
#define USB_CAMERA__HPP

#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/msg/header.hpp>
#include <fstream>
#include "yaml-cpp/yaml.h"
#include "ament_index_cpp/get_package_share_directory.hpp"

class UsbCamera : public rclcpp::Node{
public:
    UsbCamera();
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
    YAML::Node _config_yaml;
    Config _config;
    cv::VideoCapture _cap;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _iamge_pub;
    rclcpp::TimerBase::SharedPtr _pub_timer;
    cv_bridge::CvImage _cv2msg;
    void PubImgCallback();
};

#endif

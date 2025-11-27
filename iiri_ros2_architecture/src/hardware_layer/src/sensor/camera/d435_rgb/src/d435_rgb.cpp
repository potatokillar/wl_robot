#include "d435_rgb.hpp"

// 版本管理相关头文件
#ifdef HAVE_VERSION_CONFIG
#include "git_version_info.hpp" // 生成的版本信息头文件
#endif

RealsenseCamera::RealsenseCamera():rclcpp::Node("d435_rgb_node") {
    // 读取相机参数
    InitParam();

    // 打开realsense相机
    OpenCamera();

    _cv2msg.encoding = sensor_msgs::image_encodings::BGR8;
    // 设置发布器
    _img_pub = this->create_publisher<sensor_msgs::msg::Image>("image", 10);

    // 定时器，用于定时抓取和发布图像
    _timer = this->create_wall_timer(
        std::chrono::milliseconds(int(1000/_config.fps)), // 每30ms获取一帧图像
        std::bind(&RealsenseCamera::PubImgCallback, this));
    RCLCPP_INFO(this->get_logger(), "camera_node launch success");
}

void RealsenseCamera::InitParam() {
    // 读取参数
    std::string package_path = ament_index_cpp::get_package_share_directory("d435_rgb");
    YAML::Node config_yaml = YAML::LoadFile(package_path + "/config/d435_rgb.yaml");

    this->declare_parameter<bool>("undistort", false);
    this->set_parameter(rclcpp::Parameter("undistort", config_yaml["d435_rgb_node"]["ros__parameters"]["undistort"].as<bool>()));
    this->get_parameter<bool>("undistort", _config.undistort);

    this->declare_parameter<int>("device_id", 0);
    this->set_parameter(rclcpp::Parameter("device_id", config_yaml["d435_rgb_node"]["ros__parameters"]["device_id"].as<int>()));
    this->get_parameter<int>("device_id", _config.device_id);

    this->declare_parameter<int>("frame_width", 640);
    this->set_parameter(rclcpp::Parameter("frame_width", config_yaml["d435_rgb_node"]["ros__parameters"]["frame_width"].as<int>()));
    this->get_parameter<int>("frame_width", _config.frame_width);

    this->declare_parameter<int>("frame_height", 480);
    this->set_parameter(rclcpp::Parameter("frame_height", config_yaml["d435_rgb_node"]["ros__parameters"]["frame_height"].as<int>()));
    this->get_parameter<int>("frame_height", _config.frame_height);

    this->declare_parameter<int>("fps", 30);
    this->set_parameter(rclcpp::Parameter("fps", config_yaml["d435_rgb_node"]["ros__parameters"]["fps"].as<int>()));
    this->get_parameter<int>("fps", _config.fps);
    // 读取相机内参
    _config.intrinsic = cv::Mat::zeros(3, 3, CV_64F);
    _config.intrinsic.at<double>(2,2) = 0;
    _config.dist_coeff = cv::Mat::zeros(1, 5, CV_64F);

    this->declare_parameter<double>("fx", 1.0);
    this->set_parameter(rclcpp::Parameter("fx", config_yaml["d435_rgb_node"]["ros__parameters"]["fx"].as<double>()));
    this->get_parameter<double>("fx", _config.intrinsic.at<double>(0,0));

    this->declare_parameter<double>("fy", 1.0);
    this->set_parameter(rclcpp::Parameter("fy", config_yaml["d435_rgb_node"]["ros__parameters"]["fy"].as<double>()));
    this->get_parameter<double>("fy", _config.intrinsic.at<double>(1,1));

    this->declare_parameter<double>("cx", 320);
    this->set_parameter(rclcpp::Parameter("cx", config_yaml["d435_rgb_node"]["ros__parameters"]["cx"].as<double>()));
    this->get_parameter<double>("cx", _config.intrinsic.at<double>(0,2));

    this->declare_parameter<double>("cy", 240);
    this->set_parameter(rclcpp::Parameter("cy", config_yaml["d435_rgb_node"]["ros__parameters"]["cy"].as<double>()));
    this->get_parameter<double>("cy", _config.intrinsic.at<double>(1,2));

    this->declare_parameter<double>("k1", 0.0);
    this->set_parameter(rclcpp::Parameter("k1", config_yaml["d435_rgb_node"]["ros__parameters"]["k1"].as<double>()));
    this->get_parameter<double>("k1", _config.dist_coeff.at<double>(0,1));

    this->declare_parameter<double>("k2", 0.0);
    this->set_parameter(rclcpp::Parameter("k2", config_yaml["d435_rgb_node"]["ros__parameters"]["k2"].as<double>()));
    this->get_parameter<double>("k2", _config.dist_coeff.at<double>(0,1));

    this->declare_parameter<double>("p1", 0.0);
    this->set_parameter(rclcpp::Parameter("p1", config_yaml["d435_rgb_node"]["ros__parameters"]["p1"].as<double>()));
    this->get_parameter<double>("p1", _config.dist_coeff.at<double>(0,2));

    this->declare_parameter<double>("p2", 0.0);
    this->set_parameter(rclcpp::Parameter("p2", config_yaml["d435_rgb_node"]["ros__parameters"]["p2"].as<double>()));
    this->get_parameter<double>("p2", _config.dist_coeff.at<double>(0,3));

    this->declare_parameter<double>("k3", 0.0);
    this->set_parameter(rclcpp::Parameter("k3", config_yaml["d435_rgb_node"]["ros__parameters"]["k3"].as<double>()));
    this->get_parameter<double>("k3", _config.dist_coeff.at<double>(0,4));
}

void RealsenseCamera::OpenCamera() {
    rs2::config rs_cfg;
    rs_cfg.enable_stream(RS2_STREAM_COLOR, _config.frame_width, _config.frame_height, RS2_FORMAT_BGR8, _config.fps);
    // 开启realsense相机管道
    _p.start(rs_cfg);
}

void RealsenseCamera::PubImgCallback() {
        rs2::frameset frames = _p.wait_for_frames();
        rs2::frame color_frame = frames.get_color_frame();
        const int w = color_frame.as<rs2::video_frame>().get_width();
        const int h = color_frame.as<rs2::video_frame>().get_height();
        cv::Mat frame(cv::Size(w,h), CV_8UC3, (void*)color_frame.get_data());

        if (frame.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Captured an empty frame!");
            return;
        }

        if (_config.undistort)
        {
            cv::undistort(frame, frame, _config.intrinsic, _config.dist_coeff);
        }

        // 将OpenCV图像转换为ROS消息
        sensor_msgs::msg::Image img_msg;
        _cv2msg.header.stamp = this->now();
        _cv2msg.image = frame;
        _cv2msg.toImageMsg(img_msg);
        _img_pub->publish(img_msg);
}

#include "usb_camera.hpp"

// 版本管理相关头文件
#ifdef HAVE_VERSION_CONFIG
#include "git_version_info.hpp" // 生成的版本信息头文件
#endif

void UsbCamera::InitParam() {
    
    // 读取参数
    std::string package_path = ament_index_cpp::get_package_share_directory("usb_camera");
    _config_yaml = YAML::LoadFile(package_path + "/config/usb_camera.yaml");

    this->declare_parameter<int>("device_id", _config_yaml["usb_camera_node"]["ros__parameters"]["device_id"].as<int>());
    this->get_parameter<int>("device_id", _config.device_id);

    this->declare_parameter<int>("frame_width", _config_yaml["usb_camera_node"]["ros__parameters"]["frame_width"].as<int>());
    this->get_parameter<int>("frame_width", _config.frame_width);

    this->declare_parameter<int>("frame_height", _config_yaml["usb_camera_node"]["ros__parameters"]["frame_height"].as<int>());
    this->get_parameter<int>("frame_height", _config.frame_height);

    this->declare_parameter<int>("fps", _config_yaml["usb_camera_node"]["ros__parameters"]["fps"].as<int>());
    this->get_parameter<int>("fps", _config.fps);
    
    // 读取相机内参
    _config.intrinsic = cv::Mat::zeros(3, 3, CV_64F);
    _config.intrinsic.at<double>(2,2) = 0;
    _config.dist_coeff = cv::Mat::zeros(1, 5, CV_64F);
    this->declare_parameter<double>("fx", _config_yaml["usb_camera_node"]["ros__parameters"]["fx"].as<double>());
    this->declare_parameter<double>("fy", _config_yaml["usb_camera_node"]["ros__parameters"]["fy"].as<double>());
    this->declare_parameter<double>("cx", _config_yaml["usb_camera_node"]["ros__parameters"]["cx"].as<double>());
    this->declare_parameter<double>("cy", _config_yaml["usb_camera_node"]["ros__parameters"]["cy"].as<double>());
    this->declare_parameter<double>("k1", _config_yaml["usb_camera_node"]["ros__parameters"]["k1"].as<double>());
    this->declare_parameter<double>("k2", _config_yaml["usb_camera_node"]["ros__parameters"]["k2"].as<double>());
    this->declare_parameter<double>("p1", _config_yaml["usb_camera_node"]["ros__parameters"]["p1"].as<double>());
    this->declare_parameter<double>("p2", _config_yaml["usb_camera_node"]["ros__parameters"]["p2"].as<double>());
    this->declare_parameter<double>("k3", _config_yaml["usb_camera_node"]["ros__parameters"]["k3"].as<double>());
    this->declare_parameter<bool>("undistort", _config_yaml["usb_camera_node"]["ros__parameters"]["undistort"].as<bool>());
    this->get_parameter<double>("fx", _config.intrinsic.at<double>(0,0));
    this->get_parameter<double>("fy", _config.intrinsic.at<double>(1,1));
    this->get_parameter<double>("cx", _config.intrinsic.at<double>(0,2));
    this->get_parameter<double>("cy", _config.intrinsic.at<double>(1,2));
    this->get_parameter<double>("k1", _config.dist_coeff.at<double>(0,1));
    this->get_parameter<double>("k2", _config.dist_coeff.at<double>(0,1));
    this->get_parameter<double>("p1", _config.dist_coeff.at<double>(0,2));
    this->get_parameter<double>("p2", _config.dist_coeff.at<double>(0,3));
    this->get_parameter<double>("k3", _config.dist_coeff.at<double>(0,4));
    this->get_parameter<bool>("undistort", _config.undistort);
}

UsbCamera::UsbCamera():rclcpp::Node("usb_camera_node") {
    // 注册并输出版本信息
    #ifdef HAVE_VERSION_CONFIG
    REGISTER_ROS2_PACKAGE_VERSION("usb_camera");
    #endif

    // 初始化读取参数
    InitParam();

    // 打开摄像头
    OpenCamera();
    _cv2msg.encoding = sensor_msgs::image_encodings::BGR8;
    // 创建发布者
    this->_iamge_pub = this->create_publisher<sensor_msgs::msg::Image>("image", 10);
    // 创建发布图像消息定时器
    this->_pub_timer = this->create_wall_timer(
        std::chrono::milliseconds(int(1000/_config.fps)),
        std::bind(&UsbCamera::PubImgCallback, this)
    );
    // 将摄像头捕捉到的图像进行发布
}

void UsbCamera::OpenCamera() {
    _cap.open(_config.device_id, cv::CAP_V4L2);
    int temp_device_id = _config.device_id;
    if (!_cap.isOpened()) {
        RCLCPP_WARN(this->get_logger(), "Camera open with device ID %d failed", _config.device_id);
        if (_config.device_id == 0) {
            temp_device_id = 1;
        } else temp_device_id = _config.device_id - 1;
        RCLCPP_INFO(this->get_logger(), "Trying to open with device ID %d ", temp_device_id);
        _cap.open(temp_device_id, cv::CAP_V4L2);
        if (!_cap.isOpened()) {
            RCLCPP_WARN(this->get_logger(), "Camera open with device ID %d failed", temp_device_id);
            if (_config.device_id == 0) {
                temp_device_id = 2;
            } else temp_device_id = _config.device_id + 1;
            RCLCPP_INFO(this->get_logger(), "Trying to open with device ID %d ", temp_device_id);
            _cap.open(temp_device_id, cv::CAP_V4L2);
            if (!_cap.isOpened()){
                RCLCPP_WARN(this->get_logger(), "Camera open with device ID %d failed", temp_device_id);
                RCLCPP_ERROR(this->get_logger(), "Please set the correct camera device ID");
                exit(1);
            }
        }
    }
    // 打印正确的 device_id；
    RCLCPP_INFO(this->get_logger(), "Camera opened success with device ID %d ! ! !", temp_device_id);
    // 设置像素宽度;
    _cap.set(3, _config.frame_width);
    _cap.set(4, _config.frame_height);
}

void UsbCamera::PubImgCallback() {
    cv::Mat img;
    _cap.read(img);
    if (_config.undistort) {
        cv::undistort(img, img, _config.intrinsic, _config.dist_coeff);
    }
    sensor_msgs::msg::Image img_msg;
    _cv2msg.header.stamp = this->now();
    _cv2msg.image = img;
    _cv2msg.toImageMsg(img_msg);
    _iamge_pub->publish(img_msg);
    return;
}

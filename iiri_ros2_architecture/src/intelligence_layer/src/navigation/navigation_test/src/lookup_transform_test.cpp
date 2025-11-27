#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "rclcpp_action/rclcpp_action.hpp"
#include "extra/resource_tool.hpp"
#include <iostream>
#include <filesystem>
#include <fstream>
#include <sstream>


using namespace std;
using namespace std::placeholders;
using namespace std::chrono_literals;

namespace navigation_test {

class LookupTransformTest : public rclcpp::Node {
public:
    LookupTransformTest(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()) : rclcpp::Node("lookup_transform_test_node", options) {
        
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        rclcpp::sleep_for(std::chrono::seconds(1));
        lookup_transform_timer_ = this->create_wall_timer(std::chrono::milliseconds(150), std::bind(&LookupTransformTest::lookup_transform_callback, this));
        // pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", 1, std::bind(&LookupTransformTest::pose_sub_callback, this, _1));
        
    }

private:
    std::shared_ptr<rclcpp::TimerBase> lookup_transform_timer_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<rclcpp::Subscription<nav_msgs::msg::Odometry>> pose_sub_;

    void lookup_transform_callback() {
        
        geometry_msgs::msg::TransformStamped transform;
        std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
        try {
            transform = tf_buffer_->lookupTransform("map", "base_link", this->get_clock()->now(), rclcpp::Duration(std::chrono::milliseconds(200)));
        } catch (tf2::LookupException & ex) {
            RCLCPP_ERROR(
            this->get_logger(),
            "No Transform available Error looking up target frame: %s\n", ex.what());
            return;
        } catch (tf2::ExtrapolationException & ex) {
            RCLCPP_ERROR(
            this->get_logger(),
            "Extrapolation Error looking up target frame: %s\n", ex.what());
            return;
        } catch (tf2::ConnectivityException & ex) {
            RCLCPP_ERROR(
            this->get_logger(),
            "Connectivity Error looking up target frame: %s\n", ex.what());
            return;
        } 
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        RCLCPP_WARN(this->get_logger(), "get_time: %f", std::chrono::duration<double>(t1 - t0).count());

        auto now = std::chrono::system_clock::now();
        int second = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count();
        long long int millisecond = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count() % 1000000;
        RCLCPP_WARN(this->get_logger(), " time now :        %d.%lld", second, millisecond);
        RCLCPP_WARN(this->get_logger(), " transform stamp : %d.%d", transform.header.stamp.sec, transform.header.stamp.nanosec);

        return;
    }

    void pose_sub_callback(const std::shared_ptr<nav_msgs::msg::Odometry> msg) {
        
        auto get_pre = std::chrono::system_clock::now();
        auto time_now = this->get_clock()->now();
        auto t0 = std::chrono::system_clock::now();
        auto get_pos = std::chrono::system_clock::now();
        RCLCPP_WARN(this->get_logger(), "get_time: %f", std::chrono::duration<double>(get_pos - get_pre).count());

        int second = std::chrono::duration_cast<std::chrono::seconds>(t0.time_since_epoch()).count();
        long long int microsecond = std::chrono::duration_cast<std::chrono::microseconds>(t0.time_since_epoch()).count();
        long int millisecond = microsecond % 1000000;
        RCLCPP_WARN(this->get_logger(), "pose_sub_callback time now ros: %f", time_now.seconds());
        RCLCPP_WARN(this->get_logger(), "pose_sub_callback time now : %d.%ld", second, millisecond);
        RCLCPP_WARN(this->get_logger(), "msg time stamp: %d.%d", msg->header.stamp.sec, msg->header.stamp.nanosec);

    }   

};

}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<navigation_test::LookupTransformTest>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
    
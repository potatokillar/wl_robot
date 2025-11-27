#include "d435_rgb.hpp"


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RealsenseCamera>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

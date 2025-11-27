cmake_minimum_required(VERSION 3.8)
project(xiaozhi)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(robot_base REQUIRED)

add_executable(xiaozhi_node 
    src/global.cpp
    src/http.cpp
    src/main.cpp
    src/opus.cpp
    src/websocket_client.cpp
    src/xiaozhi_node.cpp 
)
target_include_directories(xiaozhi_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    include
    /usr/local/include
)
target_compile_features(xiaozhi_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(xiaozhi_node 
    curl
    ssl
    opus
    speexdsp
    OpenSSL::SSL 
    OpenSSL::Crypto
)
ament_target_dependencies(
    xiaozhi_node
    rclcpp
    interface
    std_srvs
    robot_base
    std_msgs
)

install(TARGETS xiaozhi_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # comment the line when a copyright and license is added to all source files
    set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # comment the line when this package is in a git repo and when
    # a copyright and license is added to all source files
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()

cmake_minimum_required(VERSION 3.8)
project(navigation_key)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(interface REQUIRED)

add_executable(record_traj_key_node src/record_traj_key.cpp) 
add_executable(navigation_and_speak_key_node src/navigation_and_speak_key.cpp) 
add_executable(map_3d_key_node src/map_3d_key.cpp) 

ament_target_dependencies(record_traj_key_node rclcpp geometry_msgs tf2_ros std_msgs std_srvs interface)
ament_target_dependencies(navigation_and_speak_key_node rclcpp geometry_msgs tf2_ros std_msgs std_srvs interface)
ament_target_dependencies(map_3d_key_node rclcpp geometry_msgs tf2_ros std_msgs std_srvs interface)

install(TARGETS
  record_traj_key_node map_3d_key_node navigation_and_speak_key_node
  DESTINATION
  lib/${PROJECT_NAME}
)

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

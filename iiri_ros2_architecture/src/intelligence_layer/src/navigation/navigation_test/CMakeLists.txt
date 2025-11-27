cmake_minimum_required(VERSION 3.5)
project(navigation_test)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(extra REQUIRED)
find_package(interface REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)

add_executable(path_tracker_test_node src/path_tracker_test.cpp)
add_executable(lookup_transform_test_node src/lookup_transform_test.cpp)
add_executable(traj_visual_node src/traj_visual.cpp)

ament_target_dependencies(path_tracker_test_node rclcpp rclcpp_action nav_msgs nav2_msgs extra geometry_msgs interface visualization_msgs std_srvs)
ament_target_dependencies(lookup_transform_test_node rclcpp rclcpp_action nav_msgs nav2_msgs extra geometry_msgs interface tf2_ros tf2 visualization_msgs std_srvs)
ament_target_dependencies(traj_visual_node rclcpp rclcpp_action nav_msgs nav2_msgs extra geometry_msgs interface tf2_ros tf2 visualization_msgs std_srvs sensor_msgs pcl_conversions)

target_include_directories(traj_visual_node PUBLIC ${PCL_INCLUDE_DIRS})

target_link_libraries(traj_visual_node ${PCL_LIBRARY_DIRS})

install(TARGETS path_tracker_test_node lookup_transform_test_node traj_visual_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
  launch config urdf rviz
  DESTINATION
  share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

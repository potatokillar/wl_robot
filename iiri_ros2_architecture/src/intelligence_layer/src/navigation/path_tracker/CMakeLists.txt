cmake_minimum_required(VERSION 3.5)
project(path_tracker)

set(CMAKE_BUILD_TYPE "Release")

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

# Option to enable/disable Ceres optimization
option(USE_CERES_OPTIMIZATION "Enable Ceres optimization for path tracking" OFF)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(navigation_core REQUIRED)
find_package(extra REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(interface REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)

# Conditionally find Ceres
if(USE_CERES_OPTIMIZATION)
  find_package(Ceres REQUIRED)
  message(STATUS "Ceres optimization enabled")
else()
  message(STATUS "Ceres optimization disabled")
endif()

add_library(simple_path_tracker SHARED plugins/simple_path_tracker.cpp)
add_library(basic_path_tracker SHARED plugins/basic_path_tracker.cpp)
add_library(simple_progress_checker SHARED plugins/simple_progress_checker.cpp)
add_library(simple_goal_checker SHARED plugins/simple_goal_checker.cpp)
add_library(path_tracker_node SHARED src/path_tracker_node.cpp)

ament_target_dependencies(simple_path_tracker rclcpp geometry_msgs sensor_msgs nav_msgs pluginlib rclcpp_lifecycle navigation_core nav2_msgs extra visualization_msgs)
ament_target_dependencies(basic_path_tracker rclcpp geometry_msgs sensor_msgs nav_msgs pluginlib rclcpp_lifecycle navigation_core nav2_msgs extra visualization_msgs)
ament_target_dependencies(simple_progress_checker rclcpp geometry_msgs nav_msgs pluginlib rclcpp_lifecycle navigation_core nav2_msgs extra)
ament_target_dependencies(simple_goal_checker rclcpp geometry_msgs nav_msgs pluginlib rclcpp_lifecycle navigation_core nav2_msgs extra)
ament_target_dependencies(path_tracker_node 
  rclcpp  
  geometry_msgs 
  sensor_msgs 
  nav_msgs 
  pluginlib 
  rclcpp_components 
  tf2_ros interface 
  rclcpp_lifecycle 
  rclcpp_action 
  navigation_core
  extra
  nav2_msgs
)

target_include_directories(simple_path_tracker PRIVATE include ${EIGEN3_INCLUDE_DIRS})
target_include_directories(basic_path_tracker PRIVATE include ${EIGEN3_INCLUDE_DIRS})
target_include_directories(simple_progress_checker PRIVATE include ${EIGEN3_INCLUDE_DIRS})
target_include_directories(simple_goal_checker PRIVATE include ${EIGEN3_INCLUDE_DIRS})
target_include_directories(path_tracker_node PRIVATE include ${EIGEN3_INCLUDE_DIRS})

# Conditionally link Ceres and set compile definitions
if(USE_CERES_OPTIMIZATION)
  target_link_libraries(simple_path_tracker ${EIGEN3_LIBS} Ceres::ceres)
  target_compile_definitions(simple_path_tracker PRIVATE USE_CERES_OPTIMIZATION)
else()
  target_link_libraries(simple_path_tracker ${EIGEN3_LIBS})
endif()

target_link_libraries(basic_path_tracker ${EIGEN3_LIBS})
target_link_libraries(simple_progress_checker ${EIGEN3_LIBS})
target_link_libraries(simple_goal_checker ${EIGEN3_LIBS})
target_link_libraries(path_tracker_node ${EIGEN3_LIBS})

install(TARGETS simple_path_tracker basic_path_tracker simple_progress_checker simple_goal_checker path_tracker_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(navigation_core path_tracker.xml)

rclcpp_components_register_nodes(path_tracker_node "path_tracker::PathTrackerNode")

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

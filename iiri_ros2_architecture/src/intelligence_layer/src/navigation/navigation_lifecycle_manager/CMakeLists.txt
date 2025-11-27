cmake_minimum_required(VERSION 3.5)
project(navigation_lifecycle_manager)

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(bondcpp REQUIRED)

add_library(lifecycle_manager_node SHARED src/lifecycle_manager.cpp)

ament_target_dependencies(lifecycle_manager_node rclcpp lifecycle_msgs rclcpp_action rclcpp_components std_msgs std_srvs)

target_include_directories(lifecycle_manager_node PRIVATE include)

install(TARGETS lifecycle_manager_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

rclcpp_components_register_node(
  lifecycle_manager_node
  PLUGIN "navigation_lifecycle_manager::LifecycleManager"
  EXECUTABLE lifecycle_manager_node_exec
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

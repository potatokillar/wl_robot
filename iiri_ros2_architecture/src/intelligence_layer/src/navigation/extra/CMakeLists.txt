cmake_minimum_required(VERSION 3.5)
project(extra)

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

set(CMAKE_BUILD_TYPE "Release")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(interface REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(nav_msgs REQUIRED)

include_directories(
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(collision_detect src/collision_detect.cpp)
add_library(${PROJECT_NAME} SHARED src/resource_tool.cpp src/pose_utils.cpp)

ament_target_dependencies(collision_detect rclcpp sensor_msgs geometry_msgs pcl_conversions visualization_msgs interface)
ament_target_dependencies(${PROJECT_NAME} rclcpp geometry_msgs)

target_include_directories(${PROJECT_NAME} PRIVATE include ${EIGEN3_INCLUDE_DIRS})

target_link_libraries(collision_detect ${EIGEN3_LIBS} ${PCL_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${EIGEN3_LIBS} ${PCL_LIBRARIES})

install(TARGETS 
  collision_detect 
  DESTINATION lib/${PROJECT_NAME}  
)

install(DIRECTORY 
  launch rviz  
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS ${PROJECT_NAME} DESTINATION lib)

# 别的包中要include该包内的头文件必须使用下面的编译函数导出
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

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

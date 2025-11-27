cmake_minimum_required(VERSION 3.5)
project(yd_node)

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
find_package(rmw REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
#find ydlidar_sdk package
find_package(ydlidar_sdk REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

#Include directories
include_directories(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/src
  ${YDLIDAR_SDK_INCLUDE_DIRS})

#link library directories
link_directories(${YDLIDAR_SDK_LIBRARY_DIRS})

#---------------------------------------------------------------------------------------
# generate excutable and add libraries
#---------------------------------------------------------------------------------------
add_executable(${PROJECT_NAME}
    src/yd_node.cpp)
#---------------------------------------------------------------------------------------
# link libraries
#--------------------------------------------------------------------------------------
ament_target_dependencies(${PROJECT_NAME}
    "rclcpp"
    "sensor_msgs"
    "visualization_msgs"
    "geometry_msgs"
    "std_srvs"
)

target_link_libraries(${PROJECT_NAME}
    ${YDLIDAR_SDK_LIBRARIES})

install(TARGETS 
  ${PROJECT_NAME}
  DESTINATION
  lib/${PROJECT_NAME}  
)

#---------------------------------------------------------------------------------------
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

cmake_minimum_required(VERSION 3.8)
project(motion_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic )
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(interface REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(robot_base REQUIRED)
find_package(backward_ros REQUIRED)
add_subdirectory(third-party/iiri-sdk)

# 设置文件路径
set(LIB_SRC
  
)

# 设置头文件路径
set(LIB_PRV_INC
  include/
)

set(LIB_PUB_INC
  include/motion_control
)

# 设置库
set(LIB_NAME
  iiri
)

# 四足控制节点
set(NODE1_NAME motion_control_node)
add_executable(${NODE1_NAME} src/qr.cpp ${LIB_SRC})
target_include_directories(${NODE1_NAME} PRIVATE ${LIB_PRV_INC})
target_include_directories(${NODE1_NAME} PUBLIC ${LIB_PUB_INC})
target_link_libraries(${NODE1_NAME} ${LIB_NAME})

# 添加依赖
ament_target_dependencies(${NODE1_NAME}
    rclcpp
    geometry_msgs
    interface
    sensor_msgs
    robot_base
)

#添加到install
install(TARGETS
  ${NODE1_NAME}
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

# 机械臂控制节点
set(NODE2_NAME arm_ctrl_node)
add_executable(${NODE2_NAME} src/arm.cpp ${LIB_SRC})
target_include_directories(${NODE2_NAME} PRIVATE ${LIB_PRV_INC})
target_include_directories(${NODE2_NAME} PUBLIC ${LIB_PUB_INC})
target_link_libraries(${NODE2_NAME} ${LIB_NAME})

# 添加依赖
ament_target_dependencies(${NODE2_NAME}
    rclcpp
    geometry_msgs
    interface
)

#添加到install
install(TARGETS
  ${NODE2_NAME}
  DESTINATION lib/${PROJECT_NAME}
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

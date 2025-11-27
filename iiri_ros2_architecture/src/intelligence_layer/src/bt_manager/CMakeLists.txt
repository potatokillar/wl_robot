cmake_minimum_required(VERSION 3.8)
project(bt_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(robot_base REQUIRED)

# 设置文件路径
set(LIB_SRC
  src/bt_manager_node.cpp 
  src/bt_key_control.cpp  
  src/bt_smart_follow.cpp 
  src/bt_arm_task.cpp
  src/bt_speech_recognition.cpp
  src/bt_delay.cpp
  src/bt_set_gripper.cpp
  src/bt_remote_ctrl.cpp
)

add_executable(bt_manager_node ${LIB_SRC})
target_include_directories(bt_manager_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(bt_manager_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  bt_manager_node
  interface
  rclcpp
  std_srvs
  behaviortree_cpp
  robot_base
)

install(TARGETS bt_manager_node
  DESTINATION lib/${PROJECT_NAME})

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

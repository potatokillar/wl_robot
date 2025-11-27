cmake_minimum_required(VERSION 3.8)
project(key_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic  -std=c++17)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(interface REQUIRED)
find_package(robot_base REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)

# 设置文件路径
set(LIB_SRC
  src/gamepad.cpp
  src/keyBase.cpp
)

# 设置头文件路径
set(LIB_PRV_INC
  include/
)

set(LIB_PUB_INC
  include/key_control
)

# 设置非ROS库
set(LIB_NAME
  
)

# 设置ROS库
set(NODE_LIB_ROS
  rclcpp
  geometry_msgs
  interface
  std_srvs
  std_msgs
  robot_base
)

# 节点qr_node
set(NODE_NAME qr_key_node)
add_executable(${NODE_NAME} src/keyQuad.cpp ${LIB_SRC})
target_include_directories(${NODE_NAME} PRIVATE ${LIB_PRV_INC})
target_include_directories(${NODE_NAME} PUBLIC ${LIB_PUB_INC})
target_link_libraries(${NODE_NAME} ${LIB_NAME})
# 添加依赖
ament_target_dependencies(${NODE_NAME}  ${NODE_LIB_ROS})

#添加到install
install(TARGETS ${NODE_NAME} DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

# 节点arm
set(NODE_NAME arm_key_node)
add_executable(${NODE_NAME} src/keyArm.cpp ${LIB_SRC})
target_include_directories(${NODE_NAME} PRIVATE ${LIB_PRV_INC})
target_include_directories(${NODE_NAME} PUBLIC ${LIB_PUB_INC})
target_link_libraries(${NODE_NAME} ${LIB_NAME})
ament_target_dependencies(${NODE_NAME}  ${NODE_LIB_ROS})

#添加到install
install(TARGETS
 ${NODE_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

# 节点all
set(NODE_NAME key_all_node)
add_executable(${NODE_NAME} src/keyAll.cpp ${LIB_SRC})
target_include_directories(${NODE_NAME} PRIVATE ${LIB_PRV_INC})
target_include_directories(${NODE_NAME} PUBLIC ${LIB_PUB_INC})
target_link_libraries(${NODE_NAME} ${LIB_NAME})
ament_target_dependencies(${NODE_NAME}  ${NODE_LIB_ROS})

#添加到install
install(TARGETS
 ${NODE_NAME}
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

cmake_minimum_required(VERSION 3.8)
project(smart_follow)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(interface REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(robot_base REQUIRED)

# 设置文件路径
set(LIB_SRC
  src/main.cpp
  src/serialPort.cpp
)

# 设置头文件路径
set(LIB_PRV_INC
  include/
)

set(LIB_PUB_INC
  include/smart_follow
)

# 设置库
set(LIB_NAME
  
)

# 节点
set(NODE_NAME smart_follow_node)
add_executable(${NODE_NAME} ${LIB_SRC})
target_include_directories(${NODE_NAME} PRIVATE ${LIB_PRV_INC})
target_include_directories(${NODE_NAME} PUBLIC ${LIB_PUB_INC})
target_link_libraries(${NODE_NAME} ${LIB_NAME})

# 添加依赖
ament_target_dependencies(${NODE_NAME}
    rclcpp
    interface
    geometry_msgs
    std_srvs
    std_msgs
    robot_base
)

#添加到install
install(TARGETS
 ${NODE_NAME}
  DESTINATION lib/${PROJECT_NAME}
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

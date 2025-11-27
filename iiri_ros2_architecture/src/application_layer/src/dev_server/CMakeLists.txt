cmake_minimum_required(VERSION 3.8)
project(dev_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(interface REQUIRED)
find_package(robot_base REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(OpenSSL REQUIRED)
#find_package(backward_ros REQUIRED)

# ============================================================
# 前端说明
# ============================================================
# 前端由 build_layered.sh 在宿主机上预先构建（需要 Node.js 环境）
# 如果 static/ 目录存在，将在安装阶段自动包含

# ============================================================
# 后端编译
# ============================================================
# 设置源文件路径
set(NODE_SRC
  src/ws_server.cpp
  src/http_api.cpp
  src/update_service.cpp
  src/version_info.cpp
  src/version_switch.cpp
  src/version_cleanup.cpp
)

add_executable(dev_server_node src/dev_server_node.cpp ${NODE_SRC})
target_include_directories(dev_server_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(dev_server_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  dev_server_node
  "std_msgs"
  "std_srvs"
  "interface"
  "robot_base"
  "rclcpp"
  "ament_index_cpp"
)

# 链接 OpenSSL 库（UpdateService 依赖）
target_link_libraries(dev_server_node
  OpenSSL::SSL
  OpenSSL::Crypto
)

install(TARGETS dev_server_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

# 安装前端静态文件（如果存在）
# static/ 目录由前端构建步骤生成，如果没有 Node.js 环境则跳过
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/static/)
  install(DIRECTORY static/ DESTINATION share/${PROJECT_NAME}/static)
  message(STATUS "dev_server: 将安装前端静态文件")
else()
  message(STATUS "dev_server: static/ 目录不存在，跳过前端文件安装（这是正常的，如果没有 Node.js 环境）")
endif()

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

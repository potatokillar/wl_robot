cmake_minimum_required(VERSION 3.8)
project(robot_base)

# 包含版本生成脚本 - 智能查找项目根目录
function(find_project_root VAR_NAME)
    set(SEARCH_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
    while(NOT EXISTS "${SEARCH_DIR}/cmake/generate_versions.cmake" AND NOT "${SEARCH_DIR}" STREQUAL "/")
        get_filename_component(SEARCH_DIR "${SEARCH_DIR}" DIRECTORY)
    endwhile()
    if(EXISTS "${SEARCH_DIR}/cmake/generate_versions.cmake")
        set(${VAR_NAME} "${SEARCH_DIR}" PARENT_SCOPE)
    else()
        message(FATAL_ERROR "Could not find project root with generate_versions.cmake")
    endif()
endfunction()

find_project_root(PROJECT_ROOT_DIR)
include(${PROJECT_ROOT_DIR}/cmake/generate_versions.cmake)

# 条件判断，决定通用编译选项的选择，参见build.sh
if(CMAKE_BUILD_TYPE AND (CMAKE_BUILD_TYPE STREQUAL "Debug"))
    set(COMM_FLAGS "-g -O2 -Wall -Wextra -ggdb -fPIC -Wpedantic")
    message(STATUS "Debug mode")
else()
    set(COMM_FLAGS "-O2 -Wall -Wextra -fPIC -Wpedantic")
    message(STATUS "Release mode")
endif()

set(CMAKE_C_FLAGS "-std=c99 ${COMM_FLAGS} ${ARCH_FLAGS}")
set(CMAKE_CXX_FLAGS "-std=c++17 ${COMM_FLAGS} ${ARCH_FLAGS}")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(interface REQUIRED)
find_package(backward_ros REQUIRED)

# 设置源文件路径
set(NODE_SRC
  src/cppSqlite.cpp
  src/ws_client.cpp
)

# 设置不公开的头文件路径
set(NODE_PRV_INC
  include/
)

# 设置公开头文件路径
set(NODE_PUB_INC
  include/robot_base
  ${iiri_library_INCLUDE_DIRS}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 设置非ros依赖库
set(NODE_LIB
  sqlite3
  paho-mqttpp3 
  paho-mqtt3a
)

# 设置ros依赖库
set(NODE_LIB_ROS
  rclcpp
  std_msgs
  interface
)

# 节点可执行文件
set(NODE_NAME robot_base_node)
add_executable(${NODE_NAME} src/robot_base_node.cpp ${NODE_SRC})
target_include_directories(${NODE_NAME} PRIVATE ${NODE_PRV_INC})
target_include_directories(${NODE_NAME} PUBLIC ${NODE_PUB_INC})
target_link_libraries(${NODE_NAME} ${NODE_LIB})
# 添加ros依赖
ament_target_dependencies(${NODE_NAME} ${NODE_LIB_ROS})

# 生成版本信息
generate_ros2_version_info(${NODE_NAME})

# 安装
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(TARGETS ${NODE_NAME} DESTINATION lib/${PROJECT_NAME})

# 基础库
set(LIB_NAME robot_base)
add_library(${LIB_NAME} SHARED ${NODE_SRC}) # 不含node.cpp
target_include_directories(${LIB_NAME} PRIVATE ${NODE_PRV_INC})
target_include_directories(${LIB_NAME} PUBLIC ${NODE_PUB_INC})
target_link_libraries(${LIB_NAME} ${NODE_LIB})
ament_target_dependencies(${LIB_NAME} ${NODE_LIB_ROS})

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${LIB_NAME} DESTINATION lib)
# 导出头文件路径
ament_export_include_directories(include)
ament_export_libraries(${LIB_NAME})



ament_package()

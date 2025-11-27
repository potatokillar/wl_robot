cmake_minimum_required(VERSION 3.8)
project(camera_ptz)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-w)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(interface REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(robot_base REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(OpenCV REQUIRED )
#find_package(backward_ros REQUIRED)
find_package(std_srvs REQUIRED)

pkg_check_modules(FFMPEG REQUIRED libavcodec libavformat libavutil libswscale)

# 设置文件路径
set(LIB_SRC
  src/main.cpp
  src/sdk_dahua.cpp
  src/camera.cpp
  src/dhav2wav.cpp
  src/realsense.cpp
)

# 设置头文件路径
set(LIB_PRV_INC
  
)

set(LIB_PUB_INC
  ${FFMPEG_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# 设置库
set(LIB_NAME
  dhnetsdk
  ${FFMPEG_LIBRARIES}
  swresample
  realsense2
  ${OpenCV_LIBS}
)

set(NODE_NAME camera_ptz_node)
add_executable(${NODE_NAME} ${LIB_SRC})
target_include_directories(${NODE_NAME} PRIVATE ${LIB_PRV_INC})
target_include_directories(${NODE_NAME} PUBLIC ${LIB_PUB_INC})
target_link_libraries(${NODE_NAME} ${LIB_NAME})

# 添加依赖
ament_target_dependencies(${NODE_NAME}
  "std_srvs"
  rclcpp
  interface
  sensor_msgs
  robot_base
)

#添加到install
install(TARGETS
${NODE_NAME}
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

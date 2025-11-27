cmake_minimum_required(VERSION 3.8)
project(tts)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_library(SUMMERTTS_LIBRARY NAMES summertts PATHS /usr/local/share/summertts)
if(NOT SUMMERTTS_LIBRARY)
    message(FATAL_ERROR "summertts library not found")
endif()

set(CMAKE_CXX_FLAGS "-w -O3 -fopenmp")
set(CMAKE_C_FLAGS "-w -O3 -fopenmp")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(interface REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(robot_base REQUIRED)
find_package(rclcpp REQUIRED)

# 设置头文件路径
set(LIB_PRV_INC
  include
)

set(LIB_PUB_INC
  include
)

add_executable(tts_node src/tts_node.cpp)

target_include_directories(tts_node PUBLIC
    /usr/local/include/summertts
    /usr/local/include/summertts/header
    include
    ${LIB_PUB_INC}
)
target_include_directories(tts_node PRIVATE ${LIB_PRV_INC})

target_compile_features(tts_node PUBLIC c_std_99 cxx_std_17)    # Require C99 and C++17
ament_target_dependencies(
    tts_node
    rclcpp
    std_msgs
    interface
    std_srvs
    robot_base
)

target_link_libraries(tts_node asound ${SUMMERTTS_LIBRARY})

install(TARGETS tts_node
    DESTINATION lib/${PROJECT_NAME})

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

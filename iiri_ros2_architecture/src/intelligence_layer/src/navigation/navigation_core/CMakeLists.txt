cmake_minimum_required(VERSION 3.5)
project(navigation_core)

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
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(Eigen3 REQUIRED)

set(dependencies 
  rclcpp
  nav_msgs
  geometry_msgs
  rclcpp_lifecycle
)

install(
  DIRECTORY include/
  DESTINATION include/
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

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})

ament_package()

cmake_minimum_required(VERSION 3.5)
project(quadruped)

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_VERBOSE_MAKEFILE OFF) 
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

message(STATUS "")
# 条件判断，决定通用编译选项的选择，参见build.sh
if(CMAKE_BUILD_TYPE AND (CMAKE_BUILD_TYPE STREQUAL "Debug"))
    set(COMM_FLAGS "-g -O2 -Wall -Wextra -ggdb -fPIC")
    message(STATUS "Debug mode")
else()
    set(COMM_FLAGS "-O2 -Wall -Wextra -fPIC")
    message(STATUS "Release mode")
endif()

set(CMAKE_C_FLAGS "-std=c99 ${COMM_FLAGS} ${ARCH_FLAGS}")
set(CMAKE_CXX_FLAGS "-std=c++17 ${COMM_FLAGS} ${ARCH_FLAGS}")

message(STATUS "TARGET_CPU:               ${TARGET_CPU}")
message(STATUS "CMAKE_TOOLCHAIN_FILE:     ${CMAKE_TOOLCHAIN_FILE}")
message(STATUS "PROJECT_BINARY_DIR:       ${PROJECT_BINARY_DIR}")
message(STATUS "PROJECT_SOURCE_DIR:       ${PROJECT_SOURCE_DIR}")
message(STATUS "")

# 获取Git信息
find_package(Git QUIET)
if(GIT_FOUND AND EXISTS "${PROJECT_SOURCE_DIR}/.git")
    # 获取git分支名
    execute_process(
        COMMAND ${GIT_EXECUTABLE} rev-parse --abbrev-ref HEAD
        WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
        OUTPUT_VARIABLE GIT_BRANCH
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )

    # 获取git commit hash
    execute_process(
        COMMAND ${GIT_EXECUTABLE} rev-parse HEAD
        WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
        OUTPUT_VARIABLE GIT_COMMIT
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )

    # 获取短commit hash
    execute_process(
        COMMAND ${GIT_EXECUTABLE} rev-parse --short HEAD
        WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
        OUTPUT_VARIABLE GIT_COMMIT_SHORT
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )

    # 获取远程最新的git tag
    # 首先获取远程tags
    execute_process(
        COMMAND ${GIT_EXECUTABLE} fetch --tags
        WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
        ERROR_QUIET
    )

    # 获取最新的tag（包括远程的）
    execute_process(
        COMMAND ${GIT_EXECUTABLE} tag --sort=-version:refname
        WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
        OUTPUT_VARIABLE GIT_TAGS_LIST
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
    )

    # 从tag列表中获取第一个（最新的）
    if(GIT_TAGS_LIST)
        string(REGEX REPLACE "\n.*" "" GIT_TAG "${GIT_TAGS_LIST}")
    else()
        set(GIT_TAG "v1.0.0")
    endif()
else()
    set(GIT_BRANCH "unknown")
    set(GIT_COMMIT "unknown")
    set(GIT_COMMIT_SHORT "unknown")
    set(GIT_TAG "v1.0.0")
endif()

# 生成构建时间戳
string(TIMESTAMP BUILD_TIMESTAMP "%Y%m%d_%H%M%S")

# 基于分支的版本管理逻辑
if(GIT_BRANCH STREQUAL "main")
    # main分支：使用git tag作为软件版本
    set(SOFTWARE_VERSION "${GIT_TAG}")
    set(VERSION_MODE "RELEASE")
else()
    # 非main分支：使用分支名_构建时间作为软件版本
    set(SOFTWARE_VERSION "${GIT_BRANCH}_${BUILD_TIMESTAMP}")
    set(VERSION_MODE "DEVELOPMENT")
endif()

message(STATUS "GIT_BRANCH:                ${GIT_BRANCH}")
message(STATUS "GIT_COMMIT:                ${GIT_COMMIT}")
message(STATUS "GIT_TAG:                   ${GIT_TAG}")
message(STATUS "VERSION_MODE:              ${VERSION_MODE}")
message(STATUS "SOFTWARE_VERSION:          ${SOFTWARE_VERSION}")
message(STATUS "BUILD_TIMESTAMP:           ${BUILD_TIMESTAMP}")

# 生成包含git信息的头文件
configure_file(
    ${PROJECT_SOURCE_DIR}/app/config/git_info.hpp.in
    ${PROJECT_BINARY_DIR}/app/config/git_info.hpp
    @ONLY
)

# 添加生成的头文件路径到包含目录
include_directories(${PROJECT_BINARY_DIR}/app/config)

# 添加递归子目录
add_subdirectory(app)
add_subdirectory(baseline)
add_subdirectory(control)
add_subdirectory(project) 





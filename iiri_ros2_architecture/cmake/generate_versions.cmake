# ROS2项目版本生成CMake脚本
# 基于Git分支自动生成版本号

# 获取当前脚本所在目录（在函数外部执行）
get_filename_component(VERSION_SCRIPT_DIR "${CMAKE_CURRENT_LIST_FILE}" DIRECTORY)

# ========== 修复：查找当前包所在的layer的Git仓库 ==========
function(find_layer_git_root OUTPUT_VAR)
    set(SEARCH_DIR "${CMAKE_CURRENT_SOURCE_DIR}")

    # 从当前包目录往上查找，直到找到.git目录
    while(NOT EXISTS "${SEARCH_DIR}/.git" AND NOT "${SEARCH_DIR}" STREQUAL "/")
        get_filename_component(SEARCH_DIR "${SEARCH_DIR}" DIRECTORY)
    endwhile()

    if(EXISTS "${SEARCH_DIR}/.git")
        set(${OUTPUT_VAR} "${SEARCH_DIR}" PARENT_SCOPE)
        return()
    else()
        # 如果还是找不到，设置为空
        set(${OUTPUT_VAR} "" PARENT_SCOPE)
        return()
    endif()
endfunction()

# 获取Git信息函数（修复版）
function(get_git_info)
    # 查找layer的Git仓库根目录
    find_layer_git_root(LAYER_GIT_ROOT)

    # 优先尝试读取预生成的版本信息文件（Jenkins CI/CD）
    if(LAYER_GIT_ROOT AND EXISTS "${LAYER_GIT_ROOT}/VERSION_INFO.txt")
        message(STATUS "========================================")
        message(STATUS "Using pre-generated version info from VERSION_INFO.txt")
        message(STATUS "========================================")

        # 读取版本信息文件
        file(READ "${LAYER_GIT_ROOT}/VERSION_INFO.txt" VERSION_CONTENT)

        # 解析各个字段
        string(REGEX MATCH "GIT_BRANCH=([^\n]*)" _ "${VERSION_CONTENT}")
        set(GIT_BRANCH "${CMAKE_MATCH_1}")

        string(REGEX MATCH "GIT_COMMIT=([^\n]*)" _ "${VERSION_CONTENT}")
        set(GIT_COMMIT "${CMAKE_MATCH_1}")

        string(REGEX MATCH "GIT_COMMIT_SHORT=([^\n]*)" _ "${VERSION_CONTENT}")
        set(GIT_COMMIT_SHORT "${CMAKE_MATCH_1}")

        string(REGEX MATCH "GIT_TAG=([^\n]*)" _ "${VERSION_CONTENT}")
        set(GIT_TAG "${CMAKE_MATCH_1}")

        string(REGEX MATCH "LAYER_NAME=([^\n]*)" _ "${VERSION_CONTENT}")
        set(LAYER_NAME "${CMAKE_MATCH_1}")

        message(STATUS "Loaded version info:")
        message(STATUS "  Branch: ${GIT_BRANCH}")
        message(STATUS "  Commit: ${GIT_COMMIT_SHORT}")
        message(STATUS "  Tag: ${GIT_TAG}")
        message(STATUS "  Layer: ${LAYER_NAME}")

        # 导出到父作用域
        set(GIT_BRANCH "${GIT_BRANCH}" PARENT_SCOPE)
        set(GIT_COMMIT "${GIT_COMMIT}" PARENT_SCOPE)
        set(GIT_COMMIT_SHORT "${GIT_COMMIT_SHORT}" PARENT_SCOPE)
        set(GIT_TAG "${GIT_TAG}" PARENT_SCOPE)
        set(LAYER_NAME "${LAYER_NAME}" PARENT_SCOPE)
        return()
    endif()

    # 回退到 Git 命令（本地开发环境）
    find_package(Git QUIET)
    if(GIT_FOUND AND LAYER_GIT_ROOT AND EXISTS "${LAYER_GIT_ROOT}/.git")
        message(STATUS "Found Git repository at: ${LAYER_GIT_ROOT}")
        message(STATUS "Git executable: ${GIT_EXECUTABLE}")

        # 修复Git安全目录问题（Docker环境中所有权不匹配）
        execute_process(
            COMMAND ${GIT_EXECUTABLE} config --global --add safe.directory "${LAYER_GIT_ROOT}"
            OUTPUT_QUIET
            ERROR_QUIET
        )

        # 获取git分支名
        execute_process(
            COMMAND ${GIT_EXECUTABLE} rev-parse --abbrev-ref HEAD
            WORKING_DIRECTORY "${LAYER_GIT_ROOT}"
            OUTPUT_VARIABLE GIT_BRANCH
            OUTPUT_STRIP_TRAILING_WHITESPACE
            RESULT_VARIABLE GIT_RESULT
        )
        message(STATUS "Git branch command result: ${GIT_RESULT}, branch: ${GIT_BRANCH}")

        # 获取git commit hash
        execute_process(
            COMMAND ${GIT_EXECUTABLE} rev-parse HEAD
            WORKING_DIRECTORY "${LAYER_GIT_ROOT}"
            OUTPUT_VARIABLE GIT_COMMIT
            OUTPUT_STRIP_TRAILING_WHITESPACE
            RESULT_VARIABLE GIT_RESULT
        )
        message(STATUS "Git commit command result: ${GIT_RESULT}, commit: ${GIT_COMMIT}")

        # 获取短commit hash
        execute_process(
            COMMAND ${GIT_EXECUTABLE} rev-parse --short HEAD
            WORKING_DIRECTORY "${LAYER_GIT_ROOT}"
            OUTPUT_VARIABLE GIT_COMMIT_SHORT
            OUTPUT_STRIP_TRAILING_WHITESPACE
            RESULT_VARIABLE GIT_RESULT
        )
        message(STATUS "Git short commit: ${GIT_COMMIT_SHORT}")

        # 获取最新的git tag
        execute_process(
            COMMAND ${GIT_EXECUTABLE} describe --tags --abbrev=0
            WORKING_DIRECTORY "${LAYER_GIT_ROOT}"
            OUTPUT_VARIABLE GIT_TAG
            OUTPUT_STRIP_TRAILING_WHITESPACE
            ERROR_QUIET
        )

        if(NOT GIT_TAG)
            set(GIT_TAG "v1.0.0")
        endif()

        # 获取layer名称（从路径中提取）
        get_filename_component(LAYER_NAME "${LAYER_GIT_ROOT}" NAME)
    else()
        message(WARNING "Git repository not found, using default values")
        set(GIT_BRANCH "unknown")
        set(GIT_COMMIT "unknown")
        set(GIT_COMMIT_SHORT "unknown")
        set(GIT_TAG "v1.0.0")
        set(LAYER_NAME "unknown")
    endif()

    # 导出到父作用域
    set(GIT_BRANCH "${GIT_BRANCH}" PARENT_SCOPE)
    set(GIT_COMMIT "${GIT_COMMIT}" PARENT_SCOPE)
    set(GIT_COMMIT_SHORT "${GIT_COMMIT_SHORT}" PARENT_SCOPE)
    set(GIT_TAG "${GIT_TAG}" PARENT_SCOPE)
    set(LAYER_NAME "${LAYER_NAME}" PARENT_SCOPE)
endfunction()

# 生成版本信息函数
function(generate_ros2_version_info PACKAGE_NAME)
    # 获取Git信息
    get_git_info()

    # 生成构建时间戳
    string(TIMESTAMP BUILD_TIMESTAMP "%Y-%m-%d %H:%M:%S")
    string(TIMESTAMP BUILD_TIMESTAMP_SHORT "%Y%m%d_%H%M%S")

    # 基于分支的版本管理逻辑
    if(GIT_BRANCH STREQUAL "main")
        set(SOFTWARE_VERSION "${GIT_TAG}")
        set(VERSION_MODE "RELEASE")
    else()
        set(SOFTWARE_VERSION "${GIT_BRANCH}_${BUILD_TIMESTAMP_SHORT}")
        set(VERSION_MODE "DEVELOPMENT")
    endif()

    # 输出版本信息到CMake控制台
    message(STATUS "========================================")
    message(STATUS "Package:           ${PACKAGE_NAME}")
    message(STATUS "Layer:             ${LAYER_NAME}")
    message(STATUS "Git Branch:        ${GIT_BRANCH}")
    message(STATUS "Git Commit:        ${GIT_COMMIT_SHORT}")
    message(STATUS "Git Tag:           ${GIT_TAG}")
    message(STATUS "Software Version:  ${SOFTWARE_VERSION}")
    message(STATUS "Version Mode:      ${VERSION_MODE}")
    message(STATUS "Build Timestamp:   ${BUILD_TIMESTAMP}")
    message(STATUS "========================================")

    # 生成包含版本信息的头文件
    set(VERSION_HEADER_DIR "${CMAKE_CURRENT_BINARY_DIR}/version")
    file(MAKE_DIRECTORY ${VERSION_HEADER_DIR})

    configure_file(
        ${VERSION_SCRIPT_DIR}/version_config.hpp.in
        ${VERSION_HEADER_DIR}/git_version_info.hpp
        @ONLY
    )

    # 添加编译宏，表示版本配置已启用
    if(TARGET ${PACKAGE_NAME})
        target_include_directories(${PACKAGE_NAME} PRIVATE ${VERSION_HEADER_DIR})
        target_compile_definitions(${PACKAGE_NAME} PRIVATE HAVE_VERSION_CONFIG)
    endif()

    # 导出变量供package.xml使用
    set(ROS2_PACKAGE_VERSION "${SOFTWARE_VERSION}" PARENT_SCOPE)
endfunction()

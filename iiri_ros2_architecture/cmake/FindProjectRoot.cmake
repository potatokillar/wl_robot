# FindProjectRoot.cmake
# 智能查找项目根目录的CMake模块
# 通过查找特定标记文件（如cmake目录）来确定项目根目录

function(find_project_root VAR_NAME)
    set(SEARCH_DIR "${CMAKE_CURRENT_SOURCE_DIR}")

    # 向上查找直到找到cmake目录或到达根目录
    while(NOT EXISTS "${SEARCH_DIR}/cmake" AND NOT "${SEARCH_DIR}" STREQUAL "/")
        get_filename_component(SEARCH_DIR "${SEARCH_DIR}" DIRECTORY)
    endwhile()

    if(EXISTS "${SEARCH_DIR}/cmake")
        set(${VAR_NAME} "${SEARCH_DIR}" PARENT_SCOPE)
    else()
        message(FATAL_ERROR "Could not find project root directory with cmake/ subdirectory")
    endif()
endfunction()
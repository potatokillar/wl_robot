
# 真实环境的代码
add_subdirectory(real)

# 判断是不是ubuntu18.04，默认是20.04
cmake_host_system_information(RESULT UBUNTU_VERSION QUERY OS_VERSION)
message(STATUS "${UBUNTU_VERSION}")
string(FIND "${UBUNTU_VERSION}" "18.04" substring_index)
if(NOT substring_index EQUAL -1)
    set(UBUNTU_VERSION "18.04")
else()
    set(UBUNTU_VERSION "20.04")
endif()

# 仅在x86下编译仿真程序
if(TARGET_CPU STREQUAL "x64")
    find_package(catkin QUIET)
    if(catkin_FOUND)
        if(UBUNTU_VERSION STREQUAL "18.04")
            add_subdirectory(./ros1/src/sim_bridge)
        else()
            add_subdirectory(./ros1/src)
        endif()
    endif()
endif()

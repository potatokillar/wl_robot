cmake_minimum_required(VERSION 3.5)

set(MODULE_NAME net-server)
message(STATUS "module_name:${MODULE_NAME}")

set(LIB_SRC 
    eventloop.cpp
    tcpServer.cpp
    udpServer.cpp
    multicast.cpp
    )

# 可公开的头文件路径
set(LIB_PUB_INC
    ./
    ../common
)

# 内部使用的头文件路径
set(LIB_PRV_INC
    )

# 模块需要的链接库
set(MOD_LIB
    config
    pthread
    baseline
    event
    event_pthreads
    net-common
)


# 编译成静态库
add_library(${MODULE_NAME} STATIC ${LIB_SRC})

# 添加相关选项到target
target_include_directories(${MODULE_NAME} PUBLIC ${LIB_PUB_INC})
target_include_directories(${MODULE_NAME} PRIVATE ${LIB_PRV_INC})
target_link_libraries(${MODULE_NAME} ${MOD_LIB})  

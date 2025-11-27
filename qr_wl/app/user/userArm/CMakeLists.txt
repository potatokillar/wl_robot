cmake_minimum_required(VERSION 3.5)

# sdk的API实现
set(MODULE_NAME user-arm)
message(STATUS "module_name:${MODULE_NAME}")

add_subdirectory(api)
add_subdirectory(key)
add_subdirectory(sdk)

set(LIB_SRC 
    userArmNode.cpp
    armDebugServer.cpp
    armSdk.cpp
    armKey.cpp
    )

# 可公开的头文件路径
set(LIB_PUB_INC
    ./
)

# 内部使用的头文件路径
set(LIB_PRV_INC
    )

# 模块需要的链接库
set(MOD_LIB
    user-arm-api
    user-arm-key
    user-arm-sdk
    user-dev
)


# 编译成静态库
add_library(${MODULE_NAME} STATIC ${LIB_SRC})

# 添加相关选项到target
target_include_directories(${MODULE_NAME} PUBLIC ${LIB_PUB_INC})
target_include_directories(${MODULE_NAME} PRIVATE ${LIB_PRV_INC})
target_link_libraries(${MODULE_NAME} ${MOD_LIB})  

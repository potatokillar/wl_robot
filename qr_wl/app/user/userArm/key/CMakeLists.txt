cmake_minimum_required(VERSION 3.5)
# 用户级API，包括算法，本身的API

set(MODULE_NAME user-arm-key)
message(STATUS "module_name:${MODULE_NAME}")

set(LIB_SRC 
    keyArm.cpp
    )

# 可公开的头文件路径
set(LIB_PUB_INC
    ./
)

# 内部使用的头文件路径
set(LIB_PRV_INC
    ../
    )

# 模块需要的链接库
set(MOD_LIB
    user-dev-key # 不合理todo
    user-arm-api
)


# 编译成静态库
add_library(${MODULE_NAME} STATIC ${LIB_SRC})

# 添加相关选项到target
target_include_directories(${MODULE_NAME} PUBLIC ${LIB_PUB_INC})
target_include_directories(${MODULE_NAME} PRIVATE ${LIB_PRV_INC})
target_link_libraries(${MODULE_NAME} ${MOD_LIB})  

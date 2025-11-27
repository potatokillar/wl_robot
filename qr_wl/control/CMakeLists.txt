cmake_minimum_required(VERSION 3.5)

set(MODULE_NAME control)
message(STATUS "moduleName:${MODULE_NAME}")

add_subdirectory(base) 
add_subdirectory(arm)
add_subdirectory(quadruped)


# 设置本库需要编译的源码文件
set(LIB_SRC
    control.cpp
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
    ctrl-quadruped
    ctrl-arm
    ctrl-base
)

# 编译成静态库
add_library(${MODULE_NAME} STATIC ${LIB_SRC})

# 添加相关选项到target
target_include_directories(${MODULE_NAME} PUBLIC ${LIB_PUB_INC})
target_include_directories(${MODULE_NAME} PRIVATE ${LIB_PRV_INC})
target_link_libraries(${MODULE_NAME} ${MOD_LIB}) 

cmake_minimum_required(VERSION 3.5)

set(MODULE_NAME ctrl-base)
message(STATUS "moduleName:${MODULE_NAME}")

# 设置本库需要编译的源码文件
set(LIB_SRC
    ctrlBase.cpp
    mathTools.cpp
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
    baseline
)


# 编译成静态库
add_library(${MODULE_NAME} STATIC ${LIB_SRC})

# 添加相关选项到target
target_include_directories(${MODULE_NAME} PUBLIC ${LIB_PUB_INC})
target_include_directories(${MODULE_NAME} PRIVATE ${LIB_PRV_INC})
target_link_libraries(${MODULE_NAME} ${MOD_LIB}) 

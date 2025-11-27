cmake_minimum_required(VERSION 3.5)

# 设置模块名
set(MODULE_NAME driver-canSensor)

# 设置本库需要编译的源码文件
set(LIB_SRC
    canBridge.cpp
    canDeal.cpp
    canProtocol.cpp
    )

# 设置头文件路径，其他包含本库的模块也可使用该头文件路径
set(LIB_PUB_INC
    ./
    ${PROJECT_SOURCE_DIR}/include
)

# 设置头文件路径，仅本库使用，其他包含本库的模块不得使用该头文件路径
set(LIB_PRV_INC
    
    )

# 模块需要的链接库
set(MOD_LIB
    robot
    config
    baseline
    driver-hardware
)

# 编译成静态库
add_library(${MODULE_NAME} STATIC ${LIB_SRC})

# 添加相关选项到target
target_include_directories(${MODULE_NAME} PUBLIC ${LIB_PUB_INC})
target_include_directories(${MODULE_NAME} PRIVATE ${LIB_PRV_INC})
target_link_libraries(${MODULE_NAME} ${MOD_LIB}) 


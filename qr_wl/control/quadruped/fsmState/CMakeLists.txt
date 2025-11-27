cmake_minimum_required(VERSION 3.5)

SET(MODULE_NAME fsmState)

#SET(CMAKE_PREFIX_PATH /home/wl/lib/libtorch-1.10.2 ${CMAKE_PREFIX_PATH})
#find_package(Torch REQUIRED)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

# find_package(yaml-cpp REQUIRED)
# include_directories(${YAML_CPP_INCLUDE_DIR})

# 设置本库需要编译的源码文件
set(LIB_SRC
        stateManager.cpp
        fsmLie.cpp
        fsmLieDown.cpp
        fsmStand.cpp
        fsmStandUp.cpp
        fsmWalk.cpp
        fsmState.cpp
        fsmFall.cpp
        fsmRecover.cpp
        aiWalk.cpp
        fsmHandStand.cpp
        fsmWalk4.cpp
        fsmWalk3.cpp
        fsmUpright.cpp
        fsmUpright2walk.cpp
        fsmWalk2upright.cpp
    )

# 设置头文件路径，其他包含本库的模块也可使用该头文件路径
set(LIB_PUB_INC
    ./
    ../pub
)

# 设置头文件路径，仅本库使用，其他包含本库的模块不得使用该头文件路径
set(LIB_PRV_INC
    ${EIGEN3_INCLUDE_DIRS}
    )

# 模块需要的链接库
set(LIB_PUB_LIB
    pthread
    baseline
    robot
    ctrl-tools
    ctrl-common
    cmdManager
    onnxruntime
    # yaml-cpp
)

# 必须private
set(LIB_PRV_LIB
    ${TORCH_LIBRARIES}
    Eigen3::Eigen
)

# 编译成静态库
add_library(${MODULE_NAME} STATIC ${LIB_SRC})

# 添加相关选项到target
target_include_directories(${MODULE_NAME} PUBLIC ${LIB_PUB_INC})
target_include_directories(${MODULE_NAME} PRIVATE ${LIB_PRV_INC})
target_link_libraries(${MODULE_NAME} PRIVATE ${LIB_PRV_LIB})
target_link_libraries(${MODULE_NAME} PUBLIC ${LIB_PUB_LIB})

# 为本模块添加 MIT 电机类型和设备配置的 include 路径
target_include_directories(${MODULE_NAME} PRIVATE
    ${PROJECT_SOURCE_DIR}/app/motion/mit
    ${PROJECT_SOURCE_DIR}/app/config
) 
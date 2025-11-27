cmake_minimum_required(VERSION 3.5)

# 设置模块名
set(MODULE_NAME iiri)

# 包含编译选项
if(WIN32)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
elseif(UNIX)
    set(CMAKE_CXX_FLAGS "-std=c++17 -g -Wall -rdynamic -fPIC")
    set(CMAKE_C_FLAGS "-std=c99 -g -Wall -rdynamic -fPIC")
    set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
endif()


# 包含特定的文件
if(WIN32)
    set(LIB_PUB_INC
        C:/software/boost_1_83_0
    )
else()
    # 设置boost路径，当boost路径不是系统路径时使用
    #set(BOOST_ROOT /home/lum/iros_ws/nav_env/x86/third_libs/boost-1.76.0)
    #set(Boost_NO_SYSTEM_PATHS ON)
    find_package(Boost REQUIRED COMPONENTS)
    if(Boost_FOUND)
        include_directories(${Boost_INCLUDE_DIRS})
        MESSAGE(STATUS "find boost")
    else()
        MESSAGE(STATUS "no find boost")
    endif()
endif()

# 设置本库需要编译的源码文件
set(LIB_SRC
    src/device.cpp
    src/udpClient.cpp
    src/findDevice.cpp
    src/sdkProtocolClient.cpp
    src/protocol.cpp
    src/apiDebug.cpp
    src/apiDevice.cpp
    src/apiQuadruped.cpp
    src/apiArm.cpp
    src/quadruped.cpp
    src/debugWatch.cpp
    src/quadrupedJoint.cpp
    src/quadrupedMutil.cpp
    src/armTask.cpp
    )

# 设置头文件路径，其他包含本库的模块也可使用该头文件路径
set(LIB_PUB_INC
    ./pub
    ./pub/C
    ${LIB_PUB_INC}
    ./src
)

# 设置头文件路径，仅本库使用，其他包含本库的模块不得使用该头文件路径
set(LIB_PRV_INC
    
    )

# 设置链接库，其它包含本库的模块也可用里面的符号
set(LIB_PUB_LIB
 
)

# 设置链接库，库只允许本库使用，其它库无法链接到里面的符号
if(WIN32)
    set(LIB_PRV_LIB

    )
elseif(UNIX)
    set(LIB_PRV_LIB
        pthread
        boost_system
    )
endif()

if(WIN32)
    add_library(${MODULE_NAME} STATIC ${LIB_SRC})

    # 添加相关选项到target
    target_include_directories(${MODULE_NAME} PUBLIC ${LIB_PUB_INC})
    target_include_directories(${MODULE_NAME} PRIVATE ${LIB_PRV_INC})
    target_link_libraries(${MODULE_NAME} PUBLIC ${LIB_PUB_LIB})
    target_link_libraries(${MODULE_NAME} PRIVATE ${LIB_PRV_LIB})
else()
    # 编译成动态库
    add_library(${MODULE_NAME} SHARED ${LIB_SRC})

    # 添加相关选项到target
    target_include_directories(${MODULE_NAME} PUBLIC ${LIB_PUB_INC})
    target_include_directories(${MODULE_NAME} PRIVATE ${LIB_PRV_INC})
    target_link_libraries(${MODULE_NAME} PUBLIC ${LIB_PUB_LIB}) 
    target_link_libraries(${MODULE_NAME} PRIVATE ${LIB_PRV_LIB}) 

    # 编译成静态库
    add_library(${MODULE_NAME}-static STATIC ${LIB_SRC})
    #set_target_properties(${MODULE_NAME}-static PROPERTIES OUTPUT_NAME ${MODULE_NAME})
    
    # 添加相关选项到target
    target_include_directories(${MODULE_NAME}-static PUBLIC ${LIB_PUB_INC})
    target_include_directories(${MODULE_NAME}-static PRIVATE ${LIB_PRV_INC})
    target_link_libraries(${MODULE_NAME}-static PUBLIC ${LIB_PUB_LIB}) 
    target_link_libraries(${MODULE_NAME}-static PRIVATE ${LIB_PRV_LIB}) 

    # 安装到路径中
    install(TARGETS ${MODULE_NAME} ${MODULE_NAME}-static
            LIBRARY DESTINATION lib
            ARCHIVE DESTINATION lib
            )
    install(DIRECTORY pub/ DESTINATION include/iiri)
endif()

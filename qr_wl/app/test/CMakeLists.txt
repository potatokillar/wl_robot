
# 单元测试集
cmake_minimum_required(VERSION 3.5)
PROJECT(test_app)

# 设置头文件目录
set(QR_INC
    ./
    )


# controller的测试用例
set(CONTR_SRC
    driver/TestCan.cpp
    config/TestDeviceParam.cpp
    )

# 设置源文件目录
set(QR_SRC 
    main.cpp
    ${CONTR_SRC}
    )

# 设置要链接的库
set(QR_LIB  
    config 
    robot
    network
    driver
    gtest
    )
add_executable(${PROJECT_NAME} ${QR_SRC})
target_include_directories(${PROJECT_NAME} PUBLIC ${QR_INC})
target_link_libraries(${PROJECT_NAME} PRIVATE ${QR_LIB})
target_compile_options(${PROJECT_NAME} PRIVATE "-fno-access-control") #关闭访问控制
cmake_minimum_required(VERSION 3.5)
PROJECT(qr)

# 设置头文件目录
set(QR_INC
    
    )

# 设置源文件目录
set(QR_SRC 
    main.cpp
    )

# 设置要链接的./s库
set(QR_LIB 
    app
    baseline
    )


add_executable(${PROJECT_NAME} ${QR_SRC})
target_include_directories(${PROJECT_NAME} PUBLIC ${QR_INC})
target_link_libraries(${PROJECT_NAME} PRIVATE ${QR_LIB})
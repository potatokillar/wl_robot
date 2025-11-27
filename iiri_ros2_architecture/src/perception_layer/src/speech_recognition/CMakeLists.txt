cmake_minimum_required(VERSION 3.8)
project(speech_recognition)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # add_compile_options(-Wall -Wextra -Wpedantic)
  add_compile_options(-w)
endif()
# add_compile_options(-fpermissive -Wno-narrowing -D_GLIBCXX_USE_CXX11_ABI=0)

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(interface REQUIRED)
find_package(std_srvs REQUIRED)
find_package(robot_base REQUIRED)

find_package(ZLIB REQUIRED)
# 封装C语言库
# 构建一个静态库，包含了nlsToken_c_api.h文件中声明的这些函数的C++定义，并将阿里云的库文件alibabacloud-idst-speech也编译进来，并且提供编译选项_GLIBCXX_USE_CXX11_ABI=0
add_library(nls-c-api STATIC src/nls-c-api/nlsToken_c_api.cpp)  # 编译C++库
target_include_directories(nls-c-api PUBLIC include /usr/local/include/nls)
target_link_libraries(nls-c-api PUBLIC alibabacloud-idst-speech ${ZLIB_LIBRARIES})
set_target_properties(nls-c-api PROPERTIES OUTPUT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib)
target_compile_definitions(nls-c-api PRIVATE _GLIBCXX_USE_CXX11_ABI=0)
# 构建一个动态库，C语言文件中包含了调用nlsToken_c_api.h里面的函数的一次封装，编译成C语言的库，不会再有ABI的问题
# 并且将nls-c-api以静态库的形式引入
# 后面的节点程序speech_recognition_node，以动态库的方式包含这个C语言库
# 里面的一些由于ABI问题而无法链接的函数，都替换为……(我也有点晕 2024.11.21)
add_library(nls-c-api-2 SHARED src/nls-c-api/nlsToken_c_api.c)  # 编译C库
target_include_directories(nls-c-api-2 PUBLIC include /usr/local/include/nls)
target_link_libraries(nls-c-api-2 PUBLIC nls-c-api ${ZLIB_LIBRARIES})
set_target_properties(nls-c-api-2 PROPERTIES OUTPUT_DIR ${CMAKE_SOURCE_DIR})
#添加到install
install(TARGETS
  nls-c-api-2
  DESTINATION lib
)

# 设置文件路径
file(GLOB LIB_SRC "src/*.cpp")

# 设置头文件路径
set(LIB_PRV_INC
  /usr/local/include/nls
  include
)

set(LIB_PUB_INC
  /usr/local/include/nls
  include
  ../../build_x86/build/interface/rosidl_generator_cpp
  /opt/ros/humble/include/geometry_msgs
)

# 设置库
set(LIB_NAME
  nls-c-api-2
  pthread
  asound
  rt
  z
  dl
  anl
)

set(NODE_NAME speech_recognition_node)
add_executable(${NODE_NAME} ${LIB_SRC})
target_include_directories(${NODE_NAME} PRIVATE ${LIB_PRV_INC})
target_include_directories(${NODE_NAME} PUBLIC ${LIB_PUB_INC})
target_link_libraries(${NODE_NAME} ${LIB_NAME})

# 添加依赖
ament_target_dependencies(${NODE_NAME}
  rclcpp
  interface
  std_srvs
  robot_base
)

#添加到install
install(TARGETS
${NODE_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY audio/ DESTINATION share/${PROJECT_NAME}/audio )
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

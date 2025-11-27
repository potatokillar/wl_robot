#!/bin/bash

#=============================================================================
# Docker容器内测试脚本
# 在容器内source ROS2环境并运行测试
#=============================================================================

# 设置环境
source /opt/ros/humble/setup.bash
source build_x86_shared/install/setup.bash

# 运行测试
./test_system_bringup.sh

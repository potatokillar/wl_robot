#!/bin/bash
# 务必确认修改下配置文件的位置
source iiri_env.sh

DOCKER_RUN_SHELL="source install/setup.bash && ros2 launch bringup qr_raspi.launch.py"

# 启动脚本
docker run $DOCKER_RUN_FLAG $DOCKER_IMG  bash -c "source ~/.bashrc && $DOCKER_RUN_SHELL && exit" 

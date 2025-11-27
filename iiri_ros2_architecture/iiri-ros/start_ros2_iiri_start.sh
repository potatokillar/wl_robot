#!/bin/bash

LOG_FILE=/tmp/iiri_ros_startup_debug.log
ROS_ALL_LOG=/tmp/ros2_all.log

# 第一步：设置环境变量（不记录日志，确保在主shell中执行）
export ROS_LOG_DIR=/tmp/ros2_logs
mkdir -p $ROS_LOG_DIR
# Enable coredump for debugging
ulimit -c unlimited
echo "Coredump enabled: $(ulimit -c)"
chmod 777 $ROS_LOG_DIR

export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=0
export RCUTILS_COLORIZED_OUTPUT=0
# 禁用日志缓冲，立即写入文件
export RCUTILS_LOGGING_BUFFERED_STREAM=0
export RCUTILS_LOGGING_USE_STDOUT=0

# 加载 ROS2 环境
ROS_SETUP_FILE="/opt/ros/humble/setup.bash"
if [ -f "$ROS_SETUP_FILE" ]; then
    source "$ROS_SETUP_FILE"
else
    echo "FATAL ERROR: Main ROS setup file not found at $ROS_SETUP_FILE!" > $LOG_FILE
    exit 1
fi

# 加载工作空间环境
WS_SETUP_FILE="/home/wl/autorun/iiri-ros/install/setup.bash"
if [ -f "$WS_SETUP_FILE" ]; then
    source "$WS_SETUP_FILE" 2>/dev/null
else
    echo "FATAL ERROR: Workspace setup file not found at $WS_SETUP_FILE!" >> $LOG_FILE
    exit 1
fi

# 第二步：记录环境信息
{
echo "==========================================================" 
echo "--- iiri-ros.service startup script started at $(date) ---"
echo "==========================================================" 
echo "" 
echo "Running as user: $(whoami)" 
echo "Home directory is: $HOME" 
echo "" 
echo "ROS_LOG_DIR: $ROS_LOG_DIR" 
echo "ROS_LOCALHOST_ONLY: $ROS_LOCALHOST_ONLY" 
echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID" 
echo "" 
echo "--- Environment after sourcing ---" 
echo "ROS_DISTRO: $ROS_DISTRO" 
echo "ROS_VERSION: $ROS_VERSION" 
echo "AMENT_PREFIX_PATH (first 500 chars): ${AMENT_PREFIX_PATH:0:500}" 
echo "----------------------------------" 
echo "" 
echo "Starting ros2 launch with system_bringup..." 
} > $LOG_FILE

# 清空或创建日志文件
> $ROS_ALL_LOG

# 第三步：启动 ROS2，使用 stdbuf 禁用缓冲，并用 tee 同时输出到文件和stdout
stdbuf -oL -eL ros2 launch system_bringup qr_orin.launch.py 2>&1 | tee -a $ROS_ALL_LOG

LAUNCH_EXIT_CODE=$?
echo "" >> $LOG_FILE
echo "--- ros2 launch command finished with exit code: $LAUNCH_EXIT_CODE ---" >> $LOG_FILE
echo "--- Script finished at $(date) ---" >> $LOG_FILE

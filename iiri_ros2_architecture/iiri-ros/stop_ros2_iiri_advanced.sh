#!/bin/bash

# 高级版本的ROS2关闭脚本
# 包含详细的进程管理和日志记录

LOG_FILE="/tmp/ros2_stop_$(date +%Y%m%d_%H%M%S).log"

log_message() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] $1" | tee -a "$LOG_FILE"
}

log_message "开始关闭ROS2程序..."

# 显示当前运行的ros2进程
log_message "当前运行的ROS2相关进程："
pgrep -af "ros2|qr_raspi" | tee -a "$LOG_FILE"

# 获取所有ros2相关进程的PID
ros2_pids=$(pgrep -f "ros2|qr_raspi")

if [ -z "$ros2_pids" ]; then
    log_message "未发现运行中的ROS2进程"
    exit 0
fi

log_message "发现运行中的ROS2进程，开始关闭..."

# 第一步：优雅关闭launch进程
log_message "步骤1: 优雅关闭launch进程..."
if pgrep -f "qr_raspi.launch.py" > /dev/null; then
    pkill -TERM -f "qr_raspi.launch.py"
    log_message "已发送SIGTERM信号给qr_raspi.launch.py"
    sleep 3
fi

# 第二步：关闭所有ros2 launch进程
log_message "步骤2: 关闭所有ros2 launch进程..."
if pgrep -f "ros2 launch" > /dev/null; then
    pkill -TERM -f "ros2 launch"
    log_message "已发送SIGTERM信号给所有ros2 launch进程"
    sleep 3
fi

# 第三步：关闭剩余的ros2进程
log_message "步骤3: 关闭剩余的ros2进程..."
remaining_ros2=$(pgrep -f "ros2")
if [ ! -z "$remaining_ros2" ]; then
    pkill -TERM -f "ros2"
    log_message "已发送SIGTERM信号给剩余的ros2进程"
    sleep 3
fi

# 第四步：检查并强制关闭顽固进程
log_message "步骤4: 检查顽固进程..."
stubborn_processes=$(pgrep -f "ros2|qr_raspi")
if [ ! -z "$stubborn_processes" ]; then
    log_message "发现顽固进程，使用SIGKILL强制关闭..."
    pkill -9 -f "ros2|qr_raspi"
    sleep 2
fi

# 第五步：清理ROS2守护进程
log_message "步骤5: 清理ROS2守护进程..."
if pgrep -f "_ros2_daemon" > /dev/null; then
    pkill -f "_ros2_daemon"
    log_message "已关闭ROS2守护进程"
fi

# 最终验证
log_message "最终验证..."
final_processes=$(pgrep -af "ros2|qr_raspi")
if [ -z "$final_processes" ]; then
    log_message "✅ 所有ROS2程序已成功关闭"
    echo "✅ ROS2程序关闭完成！日志文件：$LOG_FILE"
else
    log_message "⚠️ 警告：以下进程仍在运行："
    echo "$final_processes" | tee -a "$LOG_FILE"
    echo "⚠️ 请检查日志文件：$LOG_FILE"
fi

log_message "关闭操作完成"

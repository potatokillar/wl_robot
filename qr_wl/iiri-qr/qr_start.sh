#!/bin/sh
# qr 启动脚本
# systemd 服务会设置 WorkingDirectory=/home/wl/autorun/iiri-qr
# 通过符号链接机制，该路径会自动指向实际的版本目录

# 等待系统启动完成
sleep 20

# 切换到固定部署目录（通过符号链接指向实际版本）
cd /home/wl/autorun/iiri-qr || exit 1

# 启动 qr 程序，使用 qr-linkV2-3.toml 配置
exec ./qr qr-linkV2-3.toml

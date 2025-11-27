#!/bin/bash
# init_update_status.sh - 初始化 OTA 更新状态文件
# 作者: 唐文浩
# 日期: 2025-11-03
# 用途: 在 systemd 服务启动前确保状态文件存在且权限正确

STATUS_FILE="/var/run/update_status.json"
OWNER="wl:wl"
PERMISSIONS="666"

# 创建或重置状态文件
echo '{}' > "$STATUS_FILE"

# 设置正确的所有者和权限
chown "$OWNER" "$STATUS_FILE"
chmod "$PERMISSIONS" "$STATUS_FILE"

exit 0

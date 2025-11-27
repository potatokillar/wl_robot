#!/bin/bash
# Deploy update_manager to remote server

set -e

TARGET_HOST=${1:-192.168.1.54}
DEPLOY_DIR="/home/wl/autorun/update_manager"

echo "=== Deploying update_manager to $TARGET_HOST ==="

cd "$(dirname "$0")"

# Check if binary exists
if [ ! -f "build/update_manager" ]; then
    echo "Error: build/update_manager not found. Please run build_in_docker.sh first."
    exit 1
fi

# Create deployment package
echo "[1/4] Creating deployment package..."
mkdir -p deploy_tmp
cp build/update_manager deploy_tmp/
mkdir -p deploy_tmp/config
cat > deploy_tmp/config.conf << 'CONF_EOF'
# update_manager 配置文件
# 作者: 唐文浩

# 日志配置
log_dir = /var/log/update_manager
log_level = INFO

# 备份配置
backup_dir = /var/backups/iiri
max_backups = 3

# 状态文件
status_file = /tmp/update_manager_status.json

# 健康检查配置
health_check_timeout = 30
health_check_retries = 3
health_check_interval = 10

# 服务配置
ros2_service = iiri-ros.service
qr_service = iiri-qr.service

# 安装路径
ros2_install_dir = /home/wl/autorun/iiri-ros
qr_install_dir = /home/wl/autorun/iiri-qr
CONF_EOF

cd deploy_tmp
tar -czf ../update_manager_deploy.tar.gz *
cd ..
rm -rf deploy_tmp

echo "[2/4] Uploading to $TARGET_HOST..."
sshpass -p '123456' scp -o StrictHostKeyChecking=no update_manager_deploy.tar.gz wl@$TARGET_HOST:/tmp/

echo "[3/4] Extracting on remote server..."
sshpass -p '123456' ssh -o StrictHostKeyChecking=no wl@$TARGET_HOST "
    mkdir -p $DEPLOY_DIR
    cd $DEPLOY_DIR
    tar -xzf /tmp/update_manager_deploy.tar.gz
    chmod +x update_manager
    echo 'Deployment completed'
"

echo "[4/4] Verifying deployment..."
sshpass -p '123456' ssh -o StrictHostKeyChecking=no wl@$TARGET_HOST "$DEPLOY_DIR/update_manager --help" | head -10

echo ""
echo "✓ Deployment successful!"
echo "  Binary: $DEPLOY_DIR/update_manager"
echo "  Config: $DEPLOY_DIR/config.conf"

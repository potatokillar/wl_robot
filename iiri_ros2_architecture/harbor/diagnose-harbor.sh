#!/bin/bash

# Harbor 认证诊断脚本

set -e

# 配置变量
HARBOR_IP="192.168.1.93"
HARBOR_DOMAIN="harbor.local"

# 颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

log_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

echo "Harbor 认证诊断报告"
echo "===================="

# 1. 检查网络连接
log_info "1. 检查网络连接..."
if ping -c 1 -W 5 "$HARBOR_IP" &> /dev/null; then
    log_success "网络连接正常"
else
    log_error "无法连接到 $HARBOR_IP"
    exit 1
fi

# 2. 检查HTTP代理设置
log_info "2. 检查HTTP代理设置..."
if [[ -n "$http_proxy" ]]; then
    log_warning "检测到HTTP代理设置: $http_proxy"
    log_info "建议在Docker登录时设置 no_proxy 或临时禁用代理"
else
    log_success "未检测到HTTP代理"
fi

# 3. 检查hosts解析
log_info "3. 检查hosts解析..."
if grep -q "$HARBOR_DOMAIN" /etc/hosts; then
    host_entry=$(grep "$HARBOR_DOMAIN" /etc/hosts)
    log_success "hosts配置: $host_entry"
else
    log_warning "未找到 $HARBOR_DOMAIN 的hosts配置"
fi

# 4. 检查Harbor服务状态
log_info "4. 检查Harbor服务状态..."

# 使用IP访问
log_info "测试IP访问 ($HARBOR_IP):"
response_ip=$(curl --noproxy "*" -s -o /dev/null -w "%{http_code}" "http://$HARBOR_IP/" 2>/dev/null || echo "000")
if [[ "$response_ip" == "200" ]]; then
    log_success "IP访问正常 (HTTP $response_ip)"
else
    log_error "IP访问异常 (HTTP $response_ip)"
fi

# 使用域名访问
log_info "测试域名访问 ($HARBOR_DOMAIN):"
response_domain=$(curl --noproxy "*" -s -o /dev/null -w "%{http_code}" "http://$HARBOR_DOMAIN/" 2>/dev/null || echo "000")
if [[ "$response_domain" == "200" ]]; then
    log_success "域名访问正常 (HTTP $response_domain)"
else
    log_error "域名访问异常 (HTTP $response_domain)"
fi

# 5. 检查Docker Registry API
log_info "5. 检查Docker Registry API..."

# API端点检查
log_info "测试 /v2/ 端点 (IP):"
api_response_ip=$(curl --noproxy "*" -s -w "%{http_code}" "http://$HARBOR_IP/v2/" 2>/dev/null | tail -1)
if [[ "$api_response_ip" == "401" ]]; then
    log_success "API响应正常 (需要认证: $api_response_ip)"
else
    log_warning "API响应异常: $api_response_ip"
fi

log_info "测试 /v2/ 端点 (域名):"
api_response_domain=$(curl --noproxy "*" -s -w "%{http_code}" "http://$HARBOR_DOMAIN/v2/" 2>/dev/null | tail -1)
if [[ "$api_response_domain" == "401" ]]; then
    log_success "API响应正常 (需要认证: $api_response_domain)"
else
    log_warning "API响应异常: $api_response_domain"
fi

# 6. 检查认证头信息
log_info "6. 检查认证配置..."
auth_header=$(curl --noproxy "*" -s -I "http://$HARBOR_DOMAIN/v2/" 2>/dev/null | grep -i "www-authenticate" || echo "")
if [[ -n "$auth_header" ]]; then
    log_info "认证头: $auth_header"
    if echo "$auth_header" | grep -q "$HARBOR_DOMAIN"; then
        log_success "认证配置指向正确域名"
    else
        log_warning "认证配置域名不匹配"
    fi
else
    log_error "未找到认证头信息"
fi

# 7. 检查Docker客户端配置
log_info "7. 检查Docker客户端配置..."
if [[ -f "/etc/docker/daemon.json" ]]; then
    if grep -q "insecure-registries" /etc/docker/daemon.json; then
        insecure_registries=$(grep -A 5 "insecure-registries" /etc/docker/daemon.json)
        log_success "找到不安全仓库配置"
        if grep -q "$HARBOR_IP\|$HARBOR_DOMAIN" /etc/docker/daemon.json; then
            log_success "Harbor已配置为不安全仓库"
        else
            log_warning "Harbor未在不安全仓库列表中"
        fi
    else
        log_warning "未找到不安全仓库配置"
    fi
else
    log_warning "Docker daemon.json文件不存在"
fi

# 8. 用户认证建议
echo ""
log_info "8. 认证建议:"
echo "----------------------------------------"
echo "基于诊断结果，建议尝试以下操作:"
echo ""
echo "1. 直接访问Web界面确认用户账号:"
echo "   浏览器打开: http://$HARBOR_IP/"
echo ""
echo "2. 尝试不同的登录方式:"
echo "   # 使用域名 (推荐)"
echo "   echo 'your_password' | docker login $HARBOR_DOMAIN -u admin --password-stdin"
echo ""
echo "   # 使用IP"
echo "   echo 'your_password' | docker login $HARBOR_IP -u admin --password-stdin"
echo ""
echo "3. 如果代理影响访问，尝试:"
echo "   export no_proxy=\"$HARBOR_IP,$HARBOR_DOMAIN,\$no_proxy\""
echo "   echo 'your_password' | docker login $HARBOR_DOMAIN -u admin --password-stdin"
echo ""
echo "4. 检查Harbor管理界面的用户管理部分:"
echo "   - 确认用户是否存在且激活"
echo "   - 重置用户密码"
echo "   - 检查用户权限"
echo "----------------------------------------"

# 9. Docker服务状态
log_info "9. Docker服务状态:"
if systemctl is-active --quiet docker; then
    log_success "Docker服务运行正常"
else
    log_error "Docker服务未运行"
fi

echo ""
echo "诊断完成！"
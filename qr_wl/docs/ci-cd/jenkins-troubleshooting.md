# Jenkins 故障排除指南

> **文档位置**: `docs/ci-cd/jenkins-troubleshooting.md`
> **最后更新**: 2025-10-15

## Docker 相关问题

### 问题 1: Docker 镜像拉取失败

**现象**:
```
ERROR: Cannot pull image 192.168.1.93/iiri/build_x86_arm_ros1:latest
```

**原因**:
1. Harbor 仓库不可访问
2. 镜像不存在或无 latest 标签
3. Docker 无登录权限

**解决方案**:

```bash
# 1. 检查 Harbor 可访问性
curl http://192.168.1.93

# 2. 登录 Harbor
docker login 192.168.1.93

# 3. 手动拉取镜像测试
docker pull 192.168.1.93/iiri/build_x86_arm_ros1:latest

# 4. 检查镜像是否有 latest 标签
# 登录 Harbor UI: http://192.168.1.93
# 进入 iiri/build_x86_arm_ros1 仓库
# 确认 latest 标签存在
```

### 问题 2: Docker socket 权限错误

**现象**:
```
ERROR: Cannot connect to Docker socket
```

**解决方案**:

```bash
# 检查 Docker socket 权限
ls -la /var/run/docker.sock

# 修改权限
sudo chmod 666 /var/run/docker.sock
```

## Git 相关问题

### 问题 3: Git 凭据错误

**现象**:
```
ERROR: Failed to clone repository
Authentication failed
```

**解决方案**:

1. 检查 Jenkins 凭据配置
   - 进入 Jenkins → Manage Jenkins → Manage Credentials
   - 确认 `git-cred` 凭据存在且正确

2. 测试 Git 访问
   ```bash
   git clone http://root:password@192.168.1.55/ontology/qr_wl.git
   ```

3. 使用访问令牌
   - 在 GitLab 生成访问令牌
   - 更新 Jenkins 凭据使用令牌

## Orin 板连接问题

### 问题 4: SSH 连接失败

**现象**:
```
ERROR: Connection to 192.168.1.61 failed
```

**解决方案**:

```bash
# 1. 检查 Orin 板在线
ping 192.168.1.61

# 2. 测试 SSH 连接
sshpass -p '123456' ssh wl@192.168.1.61 "echo OK"

# 3. 检查 SSH 服务
ssh wl@192.168.1.61 "systemctl status sshd"

# 4. 检查防火墙
ssh wl@192.168.1.61 "sudo ufw status"
```

### 问题 5: Orin 板 Docker 环境问题

**现象**:
```
ERROR: Docker not available on Orin
```

**解决方案**:

```bash
# 检查 Orin 板 Docker
sshpass -p '123456' ssh wl@192.168.1.61 "docker --version"

# 启动 Docker 服务
sshpass -p '123456' ssh wl@192.168.1.61 "sudo systemctl start docker"

# 测试 Docker 运行
sshpass -p '123456' ssh wl@192.168.1.61 "docker ps"
```

## 构建相关问题

### 问题 6: 构建超时

**现象**:
```
BUILD TIMEOUT after 30 minutes
```

**解决方案**:

修改 Jenkinsfile 中的超时时间:
```groovy
stage('编译') {
    options {
        timeout(time: 60, unit: 'MINUTES')  // 增加到60分钟
    }
    // ...
}
```

### 问题 7: 编译失败

**现象**:
```
make: *** [target] Error 2
```

**解决方案**:

1. 查看完整日志
   - Jenkins → 构建 → Console Output

2. 本地复现
   ```bash
   ./build.sh arm
   ```

3. 检查依赖
   - 确认 Docker 镜像包含所有依赖

### 问题 8: 打包失败

**现象**:
```
ERROR: deploy_package_qr.sh failed
```

**解决方案**:

```bash
# 检查编译产物
ls -la build/arm64/output/

# 手动测试打包
./deploy_package_qr.sh arm deploy_packages --skip-build

# 检查脚本权限
chmod +x deploy_package_qr.sh
```

## 网络相关问题

### 问题 9: 网络连接超时

**现象**:
```
ERROR: Connection timed out
```

**解决方案**:

```bash
# 检查网络连通性
ping 192.168.1.93  # Jenkins
ping 192.168.1.55  # GitLab
ping 192.168.1.61  # Orin

# 检查防火墙
sudo ufw status

# 检查路由
traceroute 192.168.1.61
```

## Jenkins 服务问题

### 问题 10: Jenkins 服务不可用

**现象**:
```
ERROR: Cannot connect to Jenkins
```

**解决方案**:

```bash
# 检查 Jenkins 服务
sudo systemctl status jenkins

# 启动 Jenkins
sudo systemctl start jenkins

# 查看 Jenkins 日志
sudo journalctl -u jenkins -f
```

## 常见错误代码

| HTTP 代码 | 含义 | 解决方法 |
|-----------|------|----------|
| 401 | 认证失败 | 检查凭据 |
| 403 | 权限不足 | 检查用户权限 |
| 404 | 资源不存在 | 检查URL和任务名 |
| 500 | 服务器错误 | 查看Jenkins日志 |

## 调试技巧

### 启用详细日志

在 Jenkinsfile 中添加:
```groovy
sh 'set -x'  // 启用 shell 调试输出
```

### 保留构建目录

注释掉清理步骤进行调试:
```groovy
// deleteDir()  // 暂时注释掉
```

### 手动执行步骤

SSH 到相应机器手动执行命令:
```bash
# 在 Orin 板上手动测试
ssh wl@192.168.1.61
cd ~/jenkins_builds/qr_wl_XXX
./build.sh arm
```

## 获取帮助

遇到无法解决的问题:

1. 查看完整构建日志
2. 参考 [Jenkins 配置指南](jenkins-setup.md)
3. 参考 [Jenkins 使用指南](jenkins-usage.md)
4. 联系项目维护人员

---

**最后更新**: 2025-10-15

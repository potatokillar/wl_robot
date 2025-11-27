# Harbor 镜像仓库使用指南

> **文档位置**: `docs/infrastructure/harbor-registry.md`
> **最后更新**: 2025-10-13
> **作者**: 唐文浩

本文档说明如何使用项目的 Harbor 镜像仓库存储和拉取 Docker 镜像。

## 🎯 快速总结

Harbor 是一个企业级 Docker 镜像仓库，用于存储项目的构建镜像和部署产物。

- **访问地址**: http://192.168.1.93/
- **管理员账号**: `admin`
- **初始密码**: `Westlake1234` (首次登录后请修改)
- **版本**: Harbor v2.8.2
- **运行模式**: HTTP (局域网访问)

## 📋 服务器信息

### 基本信息

| 项目 | 值 |
|------|-----|
| **UI 地址** | http://192.168.1.93/ |
| **主机名** | harbor.local (可选使用) |
| **管理员账号** | admin |
| **初始密码** | Westlake1234 |
| **版本** | Harbor v2.8.2 |
| **数据目录** | /home/wl/soft/harbor/data |

### 组件状态

- ✅ Harbor Core - 核心服务
- ✅ Harbor Portal - Web UI
- ✅ Harbor Registry - 镜像存储
- ✅ Trivy - 漏洞扫描（已启用）
- ❌ Notary - 镜像签名（未启用）

## 🚀 客户端配置

### 步骤 1: 选择访问方式

**方式 A: 使用 IP 地址（推荐）**

直接使用 IP 地址 `192.168.1.93`，无需配置 hosts。

**方式 B: 使用域名**

使用域名 `harbor.local` 需要配置 hosts 解析：

**Linux/macOS**:
```bash
# 编辑 hosts 文件
sudo nano /etc/hosts

# 添加以下行
192.168.1.93 harbor.local
```

**Windows**:
```powershell
# 以管理员身份运行记事本
# 打开 C:\Windows\System32\drivers\etc\hosts
# 添加以下行
192.168.1.93 harbor.local
```

### 步骤 2: 配置 Docker 允许 HTTP

由于 Harbor 以 HTTP 模式运行，需要配置 Docker 允许不安全的仓库。

#### Linux 客户端

编辑 `/etc/docker/daemon.json`（如不存在则创建）：

```json
{
  "insecure-registries": ["192.168.1.93", "harbor.local"]
}
```

重启 Docker：

```bash
sudo systemctl daemon-reload
sudo systemctl restart docker
```

验证配置：

```bash
docker info | grep "Insecure Registries"
```

#### Docker Desktop (Windows/macOS)

1. 打开 Docker Desktop 设置
2. 进入 "Docker Engine" 或 "Daemon"
3. 在 JSON 配置中添加：
   ```json
   {
     "insecure-registries": ["192.168.1.93"]
   }
   ```
4. 点击 "Apply & Restart"

### 步骤 3: 防火墙配置

确保客户端可以访问服务器的 80 端口：

**在服务器上**:
```bash
# 允许 HTTP 访问
sudo ufw allow 80/tcp

# 检查防火墙状态
sudo ufw status
```

**验证连通性**:
```bash
# 从客户端测试
curl http://192.168.1.93/

# 应该看到 Harbor 页面 HTML
```

## 📦 使用 Harbor

### 登录仓库

```bash
# 使用 IP（推荐）
docker login 192.168.1.93 -u admin -p 'Westlake1234'

# 或使用域名（需 hosts 解析）
docker login harbor.local -u admin -p 'Westlake1234'
```

**首次登录后请立即修改密码！**

### 推送镜像

#### 1. 准备镜像

```bash
# 方式 A: 本地构建
docker build -t myapp:latest .

# 方式 B: 从公共仓库拉取
docker pull alpine:3.19
```

#### 2. 标记镜像

```bash
# 格式：<registry>/<project>/<image>:<tag>

# 使用 IP
docker tag alpine:3.19 192.168.1.93/iiri/alpine:3.19

# 使用域名
docker tag alpine:3.19 harbor.local/iiri/alpine:3.19
```

**注意**:
- `iiri` 是项目名称，需要先在 Harbor UI 中创建
- 项目名称必须小写
- 标签格式: `<project>/<repository>:<tag>`

#### 3. 推送到 Harbor

```bash
# 使用 IP
docker push 192.168.1.93/iiri/alpine:3.19

# 使用域名
docker push harbor.local/iiri/alpine:3.19
```

### 拉取镜像

```bash
# 使用 IP
docker pull 192.168.1.93/iiri/alpine:3.19

# 使用域名
docker pull harbor.local/iiri/alpine:3.19
```

**公开项目**：可以匿名拉取（无需登录）
**私有项目**：需要先登录且有权限

### 查看镜像列表

```bash
# 方式 1: 使用 Web UI
# 访问 http://192.168.1.93/ 并登录

# 方式 2: 使用 API
curl -u admin:Westlake1234 http://192.168.1.93/api/v2.0/projects/iiri/repositories
```

## 🌐 Web UI 使用

### 登录界面

1. 浏览器访问 http://192.168.1.93/
2. 使用管理员账号登录
3. 首次登录后，点击右上角用户名 → "Change Password" 修改密码

### 项目管理

#### 创建项目

1. 点击 "Projects" → "New Project"
2. 填写项目信息：
   - **Project Name**: `iiri` (小写，字母数字和 `-` 组成)
   - **Access Level**:
     - Public - 任何人可拉取（推荐用于基础镜像）
     - Private - 需要授权（推荐用于业务镜像）
   - **Storage Quota**: 设置存储限制（可选）
3. 点击 "OK" 创建

#### 项目配置

点击项目名称进入详情页，可以配置：

- **Members** - 添加团队成员和角色
  - Project Admin - 项目管理员
  - Developer - 可推送拉取
  - Guest - 只能拉取
- **Policy** - 镜像保留策略
- **Replication** - 镜像复制规则
- **Scanner** - 漏洞扫描配置

### 镜像管理

#### 查看镜像

1. 进入项目
2. 点击 "Repositories" 查看所有仓库
3. 点击仓库名查看标签列表

#### 扫描漏洞

1. 进入镜像详情页
2. 点击 "Scan" 按钮
3. 等待 Trivy 扫描完成
4. 查看漏洞报告（按严重程度分类）

#### 删除镜像

1. 选择要删除的标签
2. 点击 "Delete" 按钮
3. 确认删除

**注意**：删除标签后需要运行垃圾回收才能真正释放空间。

### 系统管理

进入 "Administration" 菜单：

- **Users** - 用户管理
- **Registries** - 外部仓库配置
- **Replications** - 复制任务
- **Interrogation Services** - 漏洞扫描器
- **Garbage Collection** - 垃圾回收
- **System Settings** - 系统配置

## 🔧 项目中的使用

### 构建镜像

项目中使用的 Docker 镜像：

```bash
# x86 构建镜像
192.168.1.93/iiri/build_x86_ros2:v1.4.3

# ARM 构建镜像
192.168.1.93/iiri/build_arm_ros2:v1.4.2
```

### 在 Jenkinsfile 中使用

```groovy
pipeline {
    agent {
        docker {
            image '192.168.1.93/iiri/build_x86_ros2:v1.4.3'
            args '-v /var/run/docker.sock:/var/run/docker.sock'
        }
    }

    stages {
        stage('Build') {
            steps {
                sh './build_layered.sh'
            }
        }
    }
}
```

### 在 docker-compose.yml 中使用

```yaml
version: '3'
services:
  builder:
    image: 192.168.1.93/iiri/build_x86_ros2:v1.4.3
    volumes:
      - ./:/workspace
    command: ./build.sh
```

## ⚠️ 常见问题

### 问题 1: 登录失败

**症状**:
```
Error response from daemon: Get "https://192.168.1.93/v2/": http: server gave HTTP response to HTTPS client
```

**原因**: Docker 默认使用 HTTPS，但 Harbor 以 HTTP 运行。

**解决**:
```bash
# 在 /etc/docker/daemon.json 中添加
{
  "insecure-registries": ["192.168.1.93"]
}

# 重启 Docker
sudo systemctl restart docker
```

### 问题 2: 推送被拒绝

**症状**:
```
denied: requested access to the resource is denied
```

**原因**:
- 未登录
- 项目不存在
- 没有推送权限
- 仓库名称不合法

**解决**:
1. 确认已登录：`docker login 192.168.1.93`
2. 确认项目存在：在 Web UI 中查看
3. 确认有推送权限：检查用户角色
4. 确认命名正确：`<registry>/<project>/<repo>:<tag>`

### 问题 3: 拉取/推送超时

**症状**:
```
net/http: request canceled (Client.Timeout exceeded while awaiting headers)
```

**原因**: 网络连接问题或镜像过大。

**解决**:
```bash
# 检查网络连通性
ping 192.168.1.93
telnet 192.168.1.93 80

# 检查防火墙
sudo ufw status

# 增加 Docker 超时时间
# 在 /etc/docker/daemon.json 中添加
{
  "max-concurrent-downloads": 3,
  "max-concurrent-uploads": 5
}
```

### 问题 4: 空间不足

**症状**:
```
insufficient storage
```

**原因**: Harbor 数据目录空间不足。

**解决**:
```bash
# 检查磁盘空间
df -h /home/wl/soft/harbor/data

# 运行垃圾回收（在 Harbor UI 中）
# Administration → Garbage Collection → Run Now

# 或手动清理
docker exec -it harbor-core bash
registry garbage-collect /etc/registry/config.yml
```

### 问题 5: 证书错误

**症状**:
```
x509: certificate signed by unknown authority
```

**原因**: 使用自签名证书但客户端不信任。

**解决**:
```bash
# 方式 A: 配置为不安全仓库（当前使用）
# 在 daemon.json 中添加 insecure-registries

# 方式 B: 导入 CA 证书
sudo mkdir -p /etc/docker/certs.d/192.168.1.93
sudo cp ca.crt /etc/docker/certs.d/192.168.1.93/ca.crt
sudo systemctl restart docker
```

## 🔐 安全建议

### 生产环境最佳实践

1. **修改默认密码**
   - 首次登录后立即修改 admin 密码
   - 使用强密码（大小写+数字+特殊字符）

2. **使用 HTTPS**
   - 当前为 HTTP 快速部署模式
   - 生产环境建议切换到 HTTPS
   - 参考下方 HTTPS 配置章节

3. **角色权限管理**
   - 为不同用户分配合适的角色
   - 避免过度授权
   - 定期审查用户权限

4. **启用漏洞扫描**
   - 对所有镜像进行 Trivy 扫描
   - 设置自动扫描策略
   - 阻止高危漏洞镜像部署

5. **配置镜像保留策略**
   - 自动清理旧版本镜像
   - 节省存储空间
   - 保留最近 N 个版本

## 🔒 切换到 HTTPS（可选）

### 证书位置（自签名）

```
/home/wl/soft/harbor/certs/
├── ca.crt                  # CA 证书
├── harbor.local.crt        # 服务证书
└── harbor.local.key        # 私钥
```

### 切换步骤

#### 1. 更新配置文件

编辑 `/home/wl/soft/harbor/harbor/harbor.yml`:

```yaml
# 启用 HTTPS
https:
  port: 443
  certificate: /home/wl/soft/harbor/certs/harbor.local.crt
  private_key: /home/wl/soft/harbor/certs/harbor.local.key

# HTTP 可保留，用于重定向
http:
  port: 80
```

#### 2. 应用配置

```bash
cd /home/wl/soft/harbor/harbor
sudo ./prepare
sudo ./install.sh --with-notary false --with-trivy true
```

#### 3. 客户端配置

**Linux Docker 客户端**:

```bash
# 创建证书目录
sudo mkdir -p /etc/docker/certs.d/harbor.local

# 复制 CA 证书
sudo cp /path/to/ca.crt /etc/docker/certs.d/harbor.local/ca.crt

# 重启 Docker
sudo systemctl restart docker

# 登录（使用 HTTPS）
docker login harbor.local
```

**浏览器访问**:
- 访问 https://harbor.local/
- 导入 CA 证书到浏览器（或接受自签名证书警告）

**生产环境**:
- 建议使用正式 CA 颁发的证书（如 Let's Encrypt）
- 避免自签名证书的信任问题

## 🛠️ 维护与运维

### 日常维护

#### 垃圾回收

定期清理未使用的镜像层：

1. Web UI: Administration → Garbage Collection
2. 配置定期任务（如每周一凌晨 2 点）
3. 点击 "Run Now" 立即执行

**CLI 方式**:
```bash
# 进入 Harbor Core 容器
docker exec -it harbor-core bash

# 运行垃圾回收
registry garbage-collect /etc/registry/config.yml
```

#### 备份数据

```bash
# 备份目录
/home/wl/soft/harbor/data/
├── database/       # PostgreSQL 数据库
├── registry/       # 镜像存储
├── secret/         # 密钥
└── ...

# 备份命令
sudo tar -czf harbor-backup-$(date +%Y%m%d).tar.gz \
    /home/wl/soft/harbor/data/

# 恢复（需要 Harbor 版本匹配）
sudo tar -xzf harbor-backup-20251013.tar.gz -C /
```

#### 升级 Harbor

```bash
# 1. 下载新版本
cd /home/wl/soft/harbor
wget https://github.com/goharbor/harbor/releases/download/vX.Y.Z/harbor-offline-installer-vX.Y.Z.tgz

# 2. 停止 Harbor
cd harbor
sudo docker-compose down

# 3. 备份配置和数据
sudo cp harbor.yml harbor.yml.bak
sudo tar -czf data-backup.tar.gz data/

# 4. 解压新版本
cd ..
tar -xzf harbor-offline-installer-vX.Y.Z.tgz

# 5. 迁移配置
cp harbor.yml.bak harbor/harbor.yml

# 6. 运行升级脚本
cd harbor
sudo ./install.sh --with-trivy true
```

### 监控

#### 检查服务状态

```bash
cd /home/wl/soft/harbor/harbor
docker-compose ps
```

应该看到所有服务状态为 "Up"。

#### 查看日志

```bash
# 所有服务日志
docker-compose logs

# 特定服务日志
docker-compose logs harbor-core
docker-compose logs harbor-db

# 实时跟踪
docker-compose logs -f
```

#### 磁盘使用

```bash
# 查看数据目录大小
du -sh /home/wl/soft/harbor/data/

# 查看镜像层大小
du -sh /home/wl/soft/harbor/data/registry/
```

### 性能优化

1. **配置 Redis 缓存**
   - 已默认启用
   - 可调整缓存大小

2. **使用 S3 后端存储**
   - 适合大规模部署
   - 配置 harbor.yml 中的 storage

3. **限制并发上传/下载**
   - 防止带宽占用过高
   - 在 nginx 配置中调整

## 📚 相关文档

- **[Docker 使用指南](../development/docker-guide.md)** - Docker 基础操作
- **[Jenkins CI 配置](../ci-cd/jenkins-setup.md)** - CI/CD 集成
- **[部署指南](../deployment/deployment-guide.md)** - 生产部署

### 外部资源

- [Harbor 官方文档](https://goharbor.io/docs/)
- [Harbor GitHub](https://github.com/goharbor/harbor)
- [Trivy 漏洞扫描器](https://github.com/aquasecurity/trivy)

---

**最后更新**: 2025-10-13
**版本**: v1.0
**作者**: 唐文浩

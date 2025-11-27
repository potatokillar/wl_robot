# IIRI-QR 部署包使用指南

本目录包含 IIRI-QR 二维码识别系统的完整部署包，包括编译产物和自动化部署脚本。

## 📦 目录说明

此目录是一个**自包含的部署包**,包含:

- **qr** - QR识别主程序（编译产物）
- **配置文件** - *.toml 配置文件（机器人、手臂、人员等配置）
- **ONNX模型** - onnx_model/ 目录（AI推理模型）
- **资源文件** - resource/ 目录（图片、音频等资源）
- **启动脚本** - qr_start.sh, qr_stop.sh
- **systemd 服务** - iiri-qr.service
- **部署脚本** - install.sh（一键部署）
- **运行环境** - robotrun.py, server.json
- **日志目录** - log/ 目录

> **重要提示**：`qr` 可执行文件是由编译流程自动生成的，请勿手动修改。每次运行编译后会自动更新。

## 🚀 快速开始

### 一键部署（推荐）

```bash
# 1. 解压部署包到 autorun 目录
tar -xzf ~/downloads/iiri-qr-arm-v1.0.0.tar.gz -C /home/wl/autorun/

# 2. 创建符号链接（使部署路径保持一致）
ln -snf /home/wl/autorun/iiri-qr-arm-v1.0.0 /home/wl/autorun/iiri-qr

# 3. 完整部署（安装 + 启用 + 启动）
cd /home/wl/autorun/iiri-qr
sudo ./install.sh

# 4. 查看服务状态
sudo systemctl status iiri-qr.service
```

> **💡 提示**：符号链接允许 systemd 服务使用固定路径 `/home/wl/autorun/iiri-qr`，
> 同时支持多版本并存和快速回滚。部署脚本会自动创建/更新符号链接。

### 本地开发测试

从源码编译后测试：

```bash
# 1. 编译项目（会自动输出到 iiri-qr/qr）
cd /home/wl/twh/workspace/qr_wl
./build.sh x86  # 或 ./build.sh arm

# 2. 直接在项目目录测试
cd iiri-qr
./qr_start.sh

# 或者部署到 autorun 目录
sudo ./install.sh
```

## 服务说明

### iiri-qr.service
- **功能**: QR_WL 自动运行服务
- **工作目录**: /home/wl/autorun/iiri-qr
- **启动脚本**: qr_start.sh
- **停止脚本**: qr_stop.sh
- **日志位置**: log/ 目录 + journalctl

## 使用命令

| 命令 | 说明 |
|------|------|
| sudo ./install.sh | 完整部署（安装+启用+启动） |
| sudo systemctl start iiri-qr | 启动服务 |
| sudo systemctl stop iiri-qr | 停止服务 |
| sudo systemctl restart iiri-qr | 重启服务 |
| sudo systemctl status iiri-qr | 查看服务状态 |
| ./qr_start.sh | 手动启动（前台运行） |
| ./qr_stop.sh | 手动停止 |

## 服务管理

### 查看状态
```bash
sudo systemctl status iiri-qr.service
```

### 启动/停止
```bash
# 启动
sudo systemctl start iiri-qr.service

# 停止
sudo systemctl stop iiri-qr.service

# 重启
sudo systemctl restart iiri-qr.service
```

### 查看日志
```bash
# 实时日志（systemd）
sudo journalctl -u iiri-qr.service -f

# 历史日志
sudo journalctl -u iiri-qr.service -n 50

# 应用程序日志
cat log/qr.log
tail -f log/qr.log
```

## 配置说明

### 配置文件结构

```
iiri-qr/
├── 00-base.toml              # 基础配置（必需）
├── qr-*.toml                 # 机器人配置文件
├── arm-*.toml                # 手臂配置文件
├── human-*.toml              # 人员配置文件
└── server.json               # 服务器配置
```

### 修改配置

```bash
# 编辑配置文件
vim /home/wl/autorun/iiri-qr/00-base.toml

# 重启服务使配置生效
sudo systemctl restart iiri-qr.service
```

## 故障排查

### 服务无法启动
```bash
# 检查服务状态
sudo systemctl status iiri-qr.service

# 查看详细日志
sudo journalctl -u iiri-qr.service -n 100 --no-pager

# 检查文件权限
ls -la /home/wl/autorun/iiri-qr/qr
ls -la /home/wl/autorun/iiri-qr/*.sh

# 修复权限
sudo chmod +x /home/wl/autorun/iiri-qr/qr
sudo chmod +x /home/wl/autorun/iiri-qr/*.sh
```

### 找不到 qr 可执行文件
```bash
# 检查是否存在
ls -la /home/wl/autorun/iiri-qr/qr

# 重新编译
cd /home/wl/twh/workspace/qr_wl
./build.sh arm

# 检查编译输出
ls -la iiri-qr/qr
```

### 模型文件缺失
```bash
# 检查模型目录
ls -la /home/wl/autorun/iiri-qr/onnx_model/

# 从源码复制（如果缺失）
cp -r /home/wl/twh/workspace/qr_wl/onnx_model/* /home/wl/autorun/iiri-qr/onnx_model/
```

### 权限问题
```bash
sudo chown -R wl:wl /home/wl/autorun/iiri-qr/
sudo chmod +x /home/wl/autorun/iiri-qr/*.sh
sudo chmod +x /home/wl/autorun/iiri-qr/qr
```

## 卸载服务

```bash
# 停止并禁用服务
sudo systemctl stop iiri-qr.service
sudo systemctl disable iiri-qr.service

# 删除服务文件
sudo rm /etc/systemd/system/iiri-qr.service

# 重新加载 systemd
sudo systemctl daemon-reload

# 删除部署目录（可选）
rm -rf /home/wl/autorun/iiri-qr
```

## 文件结构

```
iiri-qr/
├── README.md                          # 本文档
├── DEPLOY.md                          # 快速部署指南
├── install.sh                         # 一键安装脚本
├── VERSION.txt                        # 版本信息
├── qr                                 # QR识别主程序
├── iiri-qr.service                    # systemd 服务配置
├── qr_start.sh                        # 启动脚本
├── qr_stop.sh                         # 停止脚本
├── robotrun.py                        # Robot运行环境
├── server.json                        # 服务器配置
├── 00-base.toml                       # 基础配置
├── qr-*.toml                          # 机器人配置
├── arm-*.toml                         # 手臂配置
├── human-*.toml                       # 人员配置
├── onnx_model/                        # ONNX 模型目录
│   └── [模型文件]
├── resource/                          # 资源文件目录
│   └── [资源文件]
└── log/                               # 日志目录
    └── [日志文件]
```

## 运行时目录

```
/home/wl/autorun/
├── iiri-qr -> iiri-qr-arm-v1.0.0      # 符号链接（当前激活版本）
├── iiri-qr-arm-v1.0.0/                # 最新版本
├── iiri-qr-arm-v0.9.0/                # 上一个版本
└── iiri-qr-arm-v0.8.0/                # 更早版本
```

## 🔄 编译流程说明

### qr 可执行文件来源

`qr` 可执行文件由以下流程自动生成：

1. **本地开发编译**：
   ```bash
   cd /path/to/qr_wl
   ./build.sh arm
   # → 自动输出到 iiri-qr/qr
   ```

2. **Jenkins CI/CD**：
   ```
   1. Jenkins 自动编译 qr_wl 项目
   2. 自动输出 qr 可执行文件到 iiri-qr/
   3. 打包成 iiri-qr-{arch}-{version}.tar.gz
   4. 归档到 Jenkins 供下载
   ```

### 从 Jenkins 下载部署包

```bash
# 1. 从 Jenkins 构建页面下载部署包
wget http://admin:westlake@192.168.1.93:8080/job/qr-wl-build-ci/lastSuccessfulBuild/artifact/deploy_packages/iiri-qr-arm-v1.0.0.tar.gz

# 2. 验证校验和（如果提供）
sha256sum -c iiri-qr-arm-v1.0.0.tar.gz.sha256

# 3. 解压到目标目录
tar -xzf iiri-qr-arm-v1.0.0.tar.gz -C /home/wl/autorun/

# 4. 进入目录部署
cd /home/wl/autorun/iiri-qr-arm-v1.0.0
sudo ./install.sh
```

## 🔧 手动打包

如果需要手动创建部署包：

```bash
cd /path/to/qr_wl

# 打包 x86 版本
./deploy_package_qr.sh x86

# 打包 ARM 版本
./deploy_package_qr.sh arm

# 指定输出目录
./deploy_package_qr.sh arm custom_output_dir

# 查看生成的包
ls -lh deploy_packages/
```

## 📋 系统要求

- **操作系统**: Ubuntu 20.04 / 22.04 或 Raspberry Pi OS
- **架构**: x86_64 或 ARM64 (aarch64)
- **依赖库**:
  - ONNX Runtime
  - OpenCV
  - 其他依赖（通过 Docker 镜像提供）

## 🔗 相关链接

- **Jenkins 构建任务**: http://192.168.1.93:8080/job/qr-wl-build-ci
- **Git 仓库**: http://192.168.1.55/ontology/qr_wl.git
- **Harbor 镜像仓库**: http://192.168.1.93/

## 更新日志

### v1.0.0 (2025-10-15)
- ✅ 初始版本
- ✅ 添加自动化部署脚本
- ✅ 集成 Jenkins CI/CD 自动打包
- ✅ 添加 systemd 服务支持
- ✅ 添加版本追踪和校验和

---

**项目**: qr_wl - IIRI QR 二维码识别系统
**作者**: Westlake IIRI Team
**更新日期**: 2025-10-15

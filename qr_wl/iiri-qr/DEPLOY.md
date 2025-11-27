# IIRI-QR 部署包使用说明

## 📦 包内容

此部署包包含完整的 IIRI-QR 二维码识别系统运行环境：

- **qr** - QR识别主程序（编译产物）
- **install.sh** - 一键安装脚本 ⭐ 推荐使用
- **启动脚本** - qr_start.sh, qr_stop.sh
- **systemd 服务** - iiri-qr.service
- **配置文件** - *.toml（机器人、手臂、人员配置）
- **ONNX 模型** - onnx_model/ 目录
- **资源文件** - resource/ 目录
- **日志目录** - log/ 目录

## 🚀 极简部署（一条命令）

### Systemd 服务部署（推荐）

```bash
# 解压并进入目录
tar -xzf iiri-qr-*.tar.gz -C /home/wl/autorun/ && cd /home/wl/autorun/iiri-qr-*

# 一键安装（自动创建符号链接并部署服务）
sudo ./install.sh
```

**就这么简单！✅** install.sh 会自动完成：
1. ✅ 验证部署位置
2. ✅ 创建符号链接到 /home/wl/autorun/iiri-qr
3. ✅ 安装并启动 systemd 服务
4. ✅ 显示服务状态和管理命令

### 手动启动（开发调试）

```bash
# 1. 进入部署目录
cd /home/wl/autorun/iiri-qr

# 2. 启动系统（前台运行）
./qr_start.sh

# 3. 停止系统
./qr_stop.sh
```

## 🔄 版本管理

### 多版本并存

```bash
# 目录结构
/home/wl/autorun/
├── iiri-qr -> iiri-qr-arm-v1.0.0      # 符号链接（当前激活版本）
├── iiri-qr-arm-v1.0.0/                # 最新版本
├── iiri-qr-arm-v0.9.0/                # 上一个版本
└── iiri-qr-arm-v0.8.0/                # 更早版本
```

### 切换版本

```bash
# 停止当前服务
sudo systemctl stop iiri-qr.service

# 更新符号链接
sudo ln -snf /home/wl/autorun/iiri-qr-arm-v0.9.0 /home/wl/autorun/iiri-qr

# 启动服务
sudo systemctl start iiri-qr.service
```

### 清理旧版本

```bash
# 保留最近3个版本，删除更旧的
cd /home/wl/autorun
ls -dt iiri-qr-arm-* | tail -n +4 | xargs rm -rf

# 或手动删除指定版本
rm -rf /home/wl/autorun/iiri-qr-arm-v0.8.0
```

## 📋 前置要求

- **操作系统**: Ubuntu 20.04/22.04 或 Raspberry Pi OS
- **架构**: x86_64 或 ARM64 (aarch64)
- **权限**: sudo 权限（用于安装 systemd 服务）
- **依赖**: 已包含在部署包中，无需额外安装

## 🔧 常用命令

### 服务管理

```bash
# 启动服务
sudo systemctl start iiri-qr.service

# 停止服务
sudo systemctl stop iiri-qr.service

# 重启服务
sudo systemctl restart iiri-qr.service

# 查看状态
sudo systemctl status iiri-qr.service

# 开机自启（install.sh 自动启用）
sudo systemctl enable iiri-qr.service

# 禁用开机自启
sudo systemctl disable iiri-qr.service
```

### 日志查看

```bash
# 实时查看 systemd 服务日志
sudo journalctl -u iiri-qr.service -f

# 查看启动脚本日志
cat log/qr.log

# 查看历史日志
sudo journalctl -u iiri-qr.service -n 100

# 查看今天的日志
sudo journalctl -u iiri-qr.service --since today
```

### 配置修改

```bash
# 编辑配置文件
cd /home/wl/autorun/iiri-qr
vim 00-base.toml

# 重启服务使配置生效
sudo systemctl restart iiri-qr.service
```

## 🗂️ 文件说明

### 核心文件

| 文件 | 说明 | 是否必需 |
|------|------|---------|
| qr | 主程序可执行文件 | ✅ 必需 |
| install.sh | 一键安装脚本 | 推荐 |
| qr_start.sh | 启动脚本 | ✅ 必需 |
| qr_stop.sh | 停止脚本 | ✅ 必需 |
| iiri-qr.service | systemd 服务文件 | 推荐 |
| robotrun.py | Robot 运行环境 | ✅ 必需 |

### 配置文件

| 文件 | 说明 |
|------|------|
| 00-base.toml | 基础配置（必需） |
| qr-*.toml | 机器人配置 |
| arm-*.toml | 手臂配置 |
| human-*.toml | 人员配置 |
| server.json | 服务器配置 |

### 目录

| 目录 | 说明 |
|------|------|
| onnx_model/ | ONNX 推理模型 |
| resource/ | 资源文件（图片、音频等） |
| log/ | 日志输出目录 |

## 💡 技术说明

### 符号链接机制

- systemd 服务使用固定路径 `/home/wl/autorun/iiri-qr`
- 实际部署包含版本号（如 `iiri-qr-arm-v1.0.0`）
- 符号链接桥接固定路径和版本路径
- 支持多版本并存和快速切换

### 版本识别

部署包文件名格式：
```
iiri-qr-{架构}-{版本}.tar.gz

示例:
- iiri-qr-arm-v1.0.0.tar.gz
- iiri-qr-x86-c41edf5.tar.gz
- iiri-qr-arm-v1.0.0-dirty.tar.gz
```

### 工作目录

- **编译工作目录**: `/home/wl/twh/workspace/qr_wl`
- **部署目录**: `/home/wl/autorun/iiri-qr`
- **日志目录**: `/home/wl/autorun/iiri-qr/log`
- **服务文件**: `/etc/systemd/system/iiri-qr.service`

## 📚 更多信息

详细文档请参阅：
- **完整使用指南**: README.md
- **项目架构说明**: /home/wl/twh/workspace/qr_wl/CLAUDE.md
- **Jenkins CI/CD**: http://192.168.1.93:8080/job/qr-wl-build-ci

## 🆘 故障排查

### 服务无法启动

```bash
# 查看详细错误
sudo journalctl -u iiri-qr.service -n 50 --no-pager

# 检查文件权限
ls -la /home/wl/autorun/iiri-qr/qr

# 修复权限
sudo chmod +x /home/wl/autorun/iiri-qr/qr
sudo chmod +x /home/wl/autorun/iiri-qr/*.sh
```

### 找不到模型文件

```bash
# 检查模型目录
ls -la /home/wl/autorun/iiri-qr/onnx_model/

# 验证部署包完整性
tar -tzf iiri-qr-*.tar.gz | grep onnx_model
```

### 配置文件错误

```bash
# 验证 TOML 语法
python3 -c "import tomli; tomli.load(open('00-base.toml', 'rb'))"

# 恢复默认配置（从源码）
cp /home/wl/twh/workspace/qr_wl/config/00-base.toml /home/wl/autorun/iiri-qr/
```

## 🔗 快速链接

- **Jenkins 构建**: http://192.168.1.93:8080/job/qr-wl-build-ci
- **Git 仓库**: http://192.168.1.55/ontology/qr_wl.git
- **Harbor 镜像**: http://192.168.1.93/

---

**项目**: qr_wl - IIRI QR 二维码识别系统
**作者**: Westlake IIRI Team
**更新日期**: 2025-10-15

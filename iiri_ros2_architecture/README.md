# IIRI ROS2 分层架构主工作空间

基于ROS2 Overlay和vcstool的五层架构，提升代码组织和团队协作效率。

[![ROS2](https://img.shields.io/badge/ROS2-Humble-green)](https://docs.ros.org/en/humble/)
[![vcstool](https://img.shields.io/badge/vcstool-v0.3.0-blue)](https://github.com/dirk-thomas/vcstool)
[![License](https://img.shields.io/badge/License-Private-red)]()

## 🚀 快速开始

### 新用户标准安装（推荐）

**完整两步流程**：先准备环境，再初始化编译

```bash
# 1. 克隆主工作空间
git clone http://192.168.1.55/ontology/iiri_ros2_architecture.git
cd iiri_ros2_architecture

# 2. 安装系统依赖并导入代码
./install.sh

# 3. Harbor登录 + 拉取镜像 + 编译
./setup-and-build.sh

# 4. 验证环境
./sync.sh status
```

**说明**：
- `install.sh` 安装系统依赖（git、vcstool、Docker等）并导入代码
- `setup-and-build.sh` 完成 Harbor 登录、镜像拉取和编译

### 已有环境快速重建（跳过依赖安装）

如果系统已安装所有依赖（git、vcstool、Docker），可以跳过依赖安装：

```bash
# 1. 克隆主工作空间
git clone http://192.168.1.55/ontology/iiri_ros2_architecture.git
cd iiri_ros2_architecture

# 2. 只导入代码（跳过依赖安装）
./install.sh --no-deps

# 3. Harbor登录 + 拉取镜像 + 编译
./setup-and-build.sh
```

### 高级用户（完全手动控制）

```bash
# 1. 克隆项目
git clone http://192.168.1.55/ontology/iiri_ros2_architecture.git
cd iiri_ros2_architecture

# 2. 手动安装依赖
sudo apt-get install -y git python3-vcstool build-essential cmake

# 3. 导入代码
vcs import src < .repos

# 4. Harbor 登录
docker login 192.168.1.93

# 5. 拉取 Docker 镜像
docker pull 192.168.1.93/iiri/build_x86_ros2:latest

# 6. 编译项目
./build.sh
```

### 已有环境用户

```bash
# 更新代码
./sync.sh pull

# 更新镜像（可选，获取最新版本）
./setup-and-build.sh --pull-only --force

# 重新构建
./build.sh x86
# 或使用分层构建（推荐）
./build_layered.sh intelligence_layer  # 构建到特定层



---

## 📁 仓库结构

这是一个**主工作空间仓库**，包含管理脚本和配置文件。实际的分层代码位于独立的子仓库中：

- 📁 **iiri_ros2_architecture/** - 主工作空间（当前仓库）
  - 📄 .gitignore - Git忽略文件配置
  - 📄 .repos - vcstool配置文件
  - 📄 .repos.devel - 开发分支配置
  - 📄 .repos.main - 主分支配置文件
  - 📄 .repos.stable - 稳定版本配置
  - 🔧 build.sh - 快速编译脚本
  - 🔧 build_layered.sh - 分层编译脚本
  - 🔧 install.sh - 一键安装脚本
  - 🔧 install_completion.sh - 补全安装脚本
  - 🔧 sync.sh - 同步管理脚本
  - 🔧 release.sh - 版本发布脚本
  - 🔧 docker.sh - Docker环境脚本
  - 🔧 iiri_env.sh - 环境配置脚本
  - 📚 README.md - 项目说明（本文件）
  - 📂 **cmake/** - CMake配置目录
    - FindProjectRoot.cmake - 项目根目录查找
    - generate_versions.cmake - 版本生成脚本
    - version_config.hpp.in - 版本配置模板
  - 📂 **completions/** - 自动补全脚本目录
    - build_layered_completion.sh - 分层编译补全
    - install_completion.sh - 安装补全脚本
  - 📂 **docs/** - 文档目录（完整文档中心）
    - README.md - 文档导航中心
    - architecture/ - 架构设计文档
    - development/ - 开发指南（vcstool、构建等）
    - deployment/ - 部署指南
    - ci-cd/ - Jenkins CI/CD 文档
    - infrastructure/ - Harbor 等基础设施
    - testing/ - 测试指南和报告
    - reference/ - 参考资料和故障排除
  - 📂 **harbor/** - Harbor相关脚本目录
    - README.md - Harbor说明文档
    - diagnose-harbor.sh - Harbor诊断脚本
    - setup-harbor-client.sh - Harbor客户端设置
  - 📂 **script/** - 脚本目录
    - build_layered.sh - 分层编译脚本（Docker内使用）
  - 📂 **src/** - 分层代码（vcstool管理）
    - 📦 **core_layer/** - 核心层（独立仓库）
    - 📦 **hardware_layer/** - 硬件层（独立仓库）
    - 📦 **perception_layer/** - 感知层（独立仓库）
    - 📦 **intelligence_layer/** - 智能层（独立仓库）
    - 📦 **application_layer/** - 应用层（独立仓库）

---

## 🏗️ 分层架构说明

### 1. Core Layer (核心基础层)
- **功能**: 基础消息定义、第三方工具和系统启动
- **仓库**: [iiri-core-layer](http://192.168.1.55/ontology/iiri-core-layer)
- **依赖**: ROS2 基础环境
- **包含**: `interface`, `third_party`, `system_bringup`
- **重要**: `system_bringup` 支持分层和平台特定启动，解决了原 bringup 在应用层导致必须编译5层才能运行的问题

### 2. Hardware Layer (硬件抽象层)
- **功能**: 硬件驱动和底层接口
- **仓库**: [iiri-hardware-layer](http://192.168.1.55/ontology/iiri-hardware-layer)
- **依赖**: Core Layer
- **包含**: `motion_control`, `robot_base`, `sensor`

### 3. Perception Layer (感知处理层)
- **功能**: 感知数据处理
- **仓库**: [iiri-perception-layer](http://192.168.1.55/ontology/iiri-perception-layer)
- **依赖**: Hardware Layer
- **包含**: `camera_ptz`, `speaker`, `speech_recognition`, `tts`

### 4. Intelligence Layer (智能导航层)
- **功能**: 智能决策和导航
- **仓库**: [iiri-intelligence-layer](http://192.168.1.55/ontology/iiri-intelligence-layer)
- **依赖**: Perception Layer
- **包含**: `navigation`, `bt_manager`, `smart_follow`, `xiaozhi`

### 5. Application Layer (应用通信层)
- **功能**: 应用逻辑和网络通信
- **仓库**: [iiri-application-layer](http://192.168.1.55/ontology/iiri-application-layer)
- **依赖**: Intelligence Layer
- **包含**: `bringup`, `dev_server`, `key_control`, `record`, `remote_ctrl`

---

## 🔧 管理脚本

### setup-and-build.sh - 一键初始化和编译脚本（推荐新人）

**推荐新人使用**的一键脚本，自动完成从 Harbor 登录到编译的全流程：

```bash
# 一键完成所有步骤（推荐）
./setup-and-build.sh

# 只拉取镜像，不编译
./setup-and-build.sh --pull-only

# 强制拉取最新镜像
./setup-and-build.sh --pull-only --force

# 跳过某些步骤
./setup-and-build.sh --skip-login      # 跳过 Harbor 登录
./setup-and-build.sh --skip-import     # 跳过代码导入

# 查看所有选项
./setup-and-build.sh --help
```

**功能**：
- ✅ 检查 Docker 环境
- ✅ Harbor 自动登录
- ✅ 自动拉取 Docker 镜像
- ✅ 导入代码（如果缺失）
- ✅ 执行编译
- ✅ 智能镜像管理（存在则跳过，缺失则拉取）

**使用场景**：
- 🆕 新人首次设置环境
- 🔄 更新 Docker 镜像到最新版本
- 🚀 快速重新编译项目

---

### install.sh - 系统环境安装脚本

系统依赖和代码导入脚本，适合首次环境准备或配置切换：

```bash
# 标准安装（依赖 + 代码导入）
./install.sh

# 使用开发分支
./install.sh --config devel

# 跳过依赖安装（只导入代码）
./install.sh --no-deps

# 查看帮助
./install.sh --help
```

**功能**：
- ✅ 安装系统依赖（apt-get）
- ✅ 配置 vcstool 环境
- ✅ 导入所有分层代码
- ✅ 支持配置切换（main/devel/stable）
- ❌ 不包含 Docker 镜像管理
- ❌ 不包含编译

**完成后**：运行 `./setup-and-build.sh` 完成镜像拉取和编译

### sync.sh - 同步管理脚本

日常开发的主要工具：

```bash
# 查看所有仓库状态
./sync.sh status

# 更新所有代码
./sync.sh pull

# 推送所有仓库改动（新增功能）
./sync.sh push                     # 交互式推送（会询问是否更新主仓库指针）
./sync.sh push --update-main       # 自动推送并更新主仓库指针（适合CI/CD）

# 首次导入代码
./sync.sh import

# 切换配置
./sync.sh switch devel     # 开发分支
./sync.sh switch stable    # 稳定版本
./sync.sh switch main      # 主分支

# 清理代码（重新开始）
./sync.sh clean
```

**push 命令说明**：
- `./sync.sh push` - 推送所有子仓库改动，并智能检测主仓库子模块指针是否需要更新
  - 自动使用 `vcs push` 推送所有子仓库
  - 检测到子模块指针变化时，交互式询问是否更新主仓库
  - 显示变化的子模块清单
  - 提供手动操作提示

- `./sync.sh push --update-main` - 自动化推送（无需人工确认）
  - 推送所有子仓库改动
  - 自动更新主仓库子模块指针
  - 自动生成规范的 commit message
  - 包含作者署名和变更子模块列表
  - 适合 CI/CD 自动化流程

**使用场景**：
```bash
# 场景1：日常开发推送
cd src/intelligence_layer
git commit -m "feat: 优化导航算法"
cd ../..
./sync.sh push                     # 交互式确认推送

# 场景2：自动化部署
./sync.sh push --update-main       # 一键完成所有推送操作
```

### release.sh - 版本发布脚本

用于创建和管理项目版本：

```bash
# 验证发布准备状态
./release.sh validate

# 查看各层版本状态
./release.sh status

# 创建新版本发布
./release.sh create v1.2.0

# 列出所有版本
./release.sh list
```

### docker.sh - Docker环境脚本

Docker环境管理（保留原有功能并增强vcstool支持）：

```bash
# 启动Docker并配置vcstool环境
./docker.sh setup

# 启动普通容器
./docker.sh run

# 进入现有容器
./docker.sh exec <容器ID>
```

---

## 📚 文档资源

### 团队必读文档

- **[vcstool 使用指南](./docs/development/vcstool-guide.md)** - 详细的 vcstool 命令和使用方法
- **[vcstool 团队协作指南](./docs/development/vcstool-team-guide.md)** - 团队多人协作最佳实践

### 各层仓库文档

每个分层仓库都有独立的README和文档：
- [Core Layer文档](http://192.168.1.55/ontology/iiri-core-layer)
- [Hardware Layer文档](http://192.168.1.55/ontology/iiri-hardware-layer)
- [Perception Layer文档](http://192.168.1.55/ontology/iiri-perception-layer)
- [Intelligence Layer文档](http://192.168.1.55/ontology/iiri-intelligence-layer)
- [Application Layer文档](http://192.168.1.55/ontology/iiri-application-layer)

---

## 🔨 构建说明

### 使用 Docker 编译（推荐）

#### 快速编译所有层（新增build.sh）
```bash
# 简化的编译命令 - 自动编译所有5个层
./build.sh                         # 编译所有层（自动检测架构）
./build.sh x86                     # x86 架构编译所有层
./build.sh arm                     # ARM 架构编译所有层
./build.sh -c x86                  # 清理后编译所有层
./build.sh help                    # 查看帮助信息
```

#### 分层编译（精细控制）
```bash
# 基础编译（默认禁用 Ceres 优化）
./build_layered.sh                              # 编译所有层（自动检测架构）
./build_layered.sh intelligence_layer           # 编译到智能导航层
./build_layered.sh application_layer            # 编译到应用通信层
./build_layered.sh -c intelligence_layer        # 清理后编译到智能导航层

# 启用 Ceres 优化编译
./build_layered.sh --ceres                      # 编译所有层并启用 Ceres
./build_layered.sh --ceres intelligence_layer   # 编译到智能导航层并启用 Ceres
./build_layered.sh --ceres application_layer    # 编译到应用通信层并启用 Ceres
./build_layered.sh -c --ceres intelligence_layer # 清理后编译并启用 Ceres

# 便捷脚本（自动启用 Ceres）
./build_layered_ceres.sh                        # 自动启用 Ceres 编译所有层
./build_layered_ceres.sh intelligence_layer     # 自动启用 Ceres 编译到智能导航层
./build_layered_ceres.sh -c application_layer   # 清理后启用 Ceres 编译到应用通信层

# 帮助信息
./build_layered.sh help                         # 显示详细帮助
```

**重要说明**：
- `--ceres` 参数用于启用 path_tracker 的 Ceres 优化功能
- Intelligence Layer 包含 path_tracker，建议在需要高精度路径跟踪时启用 Ceres
- Application Layer 依赖 Intelligence Layer，编译时需保持 Ceres 设置一致性

### Docker 环境

项目使用以下 Docker 镜像：
- **x86架构**: `192.168.1.93/iiri/build_x86_ros2:latest`
- **ARM架构**: `192.168.1.93/iiri/build_arm_ros2:latest`

**镜像管理**：
- 使用 `latest` 标签自动获取最新版本
- 一键拉取镜像: `./setup-and-build.sh --pull-only`
- 强制更新镜像: `./setup-and-build.sh --pull-only --force`
- 手动登录: `docker login 192.168.1.93`

### 构建产物

编译后会生成以下目录：
- `build_x86_*` 或 `build_arm_*` - 各层构建目录
- 每层都有独立的 `build`、`install`、`log` 目录

### 常用构建场景

#### 场景一：快速开发（推荐新手）
```bash
# 简单一键编译 - 最快捷的方式
./build.sh                           # 编译所有层
./build.sh -c                        # 清理后编译所有层
```

#### 场景二：开发 Intelligence Layer（路径跟踪功能）
```bash
# 基础开发（快速编译）
./build_layered.sh intelligence_layer

# 高精度测试（启用 Ceres 优化）
./build_layered.sh --ceres intelligence_layer
```

#### 场景三：完整系统部署
```bash
# 生产环境（启用所有优化）
./build_layered_ceres.sh

# 或者使用完整参数
./build_layered.sh --ceres application_layer
```

#### 场景四：调试和开发
```bash
# 清理重建（基础模式）
./build_layered.sh -c intelligence_layer

# 清理重建（优化模式）
./build_layered.sh -c --ceres application_layer
```

### 手动分层编译

```bash
# 1. 进入 Docker 容器
./docker.sh

# 2. 在容器内运行分层编译脚本
# 基础编译（默认禁用 Ceres）
./script/build_layered.sh                          # 编译所有层
./script/build_layered.sh -c                       # 清理后编译所有层
./script/build_layered.sh intelligence_layer       # 编译到智能导航层（包括依赖）
./script/build_layered.sh -c intelligence_layer    # 清理后编译到智能导航层

# 启用 Ceres 优化编译
./script/build_layered.sh --ceres                  # 编译所有层并启用 Ceres
./script/build_layered.sh --ceres intelligence_layer # 编译到智能导航层并启用 Ceres
./script/build_layered.sh --ceres application_layer  # 编译到应用通信层并启用 Ceres
./script/build_layered.sh -c --ceres intelligence_layer # 清理后编译并启用 Ceres

# 或者在容器内手动逐层编译（不推荐，仅用于调试）
cd core_layer && source setup.bash && colcon build
cd ../hardware_layer && source setup.bash && colcon build
cd ../perception_layer && source setup.bash && colcon build
cd ../intelligence_layer && source setup.bash && colcon build
cd ../application_layer && source setup.bash && colcon build
```

### 分层编译脚本说明

`script/build_layered.sh` 是容器内使用的分层编译脚本：

- **功能**: 按正确依赖顺序编译指定层次
- **参数**:
  - 无参数: 编译所有层
  - `-c`: 清理构建缓存
  - `--ceres`: 启用 Ceres 优化（用于 path_tracker 高精度路径跟踪）
  - `layer_name`: 指定要编译的层（会自动编译其依赖层）
    - `core_layer`: 只编译核心层
    - `hardware_layer`: 编译核心层 + 硬件层
    - `perception_layer`: 编译到感知层
    - `intelligence_layer`: 编译到智能层（包含 path_tracker）
    - `application_layer`: 编译所有层
- **组合使用**: 
  - `-c core_layer` 清理后只编译核心层
  - `--ceres intelligence_layer` 启用 Ceres 编译到智能层
  - `-c --ceres application_layer` 清理后启用 Ceres 编译所有层
- **架构**: 自动检测 x86_64/aarch64 并使用对应的构建目录

**Ceres 优化说明**：
- Ceres 优化主要用于 Intelligence Layer 中的 path_tracker 包
- 启用后可提供更高精度的路径跟踪和优化算法
- Application Layer 编译时需与 Intelligence Layer 保持 Ceres 设置一致

### 🎯 自动补全功能

为了提升开发效率，项目提供了 `build_layered.sh` 脚本的 bash 自动补全功能：

#### 安装自动补全
```bash
# 一键安装自动补全功能
./install_completion.sh
```

#### 使用自动补全
安装后，您可以使用 TAB 键进行智能补全：

```bash
# 补全层名称
./script/build_layered.sh core<TAB>     # 自动补全为 core_layer
./script/build_layered.sh intel<TAB>    # 自动补全为 intelligence_layer

# 补全参数
./script/build_layered.sh -<TAB>        # 显示 -c, --ceres, help
./script/build_layered.sh --<TAB>       # 显示 --ceres

# 补全架构选项
./script/build_layered.sh x<TAB>        # 自动补全为 x86
./script/build_layered.sh ar<TAB>       # 自动补全为 arm

# 组合补全
./script/build_layered.sh -c <TAB>      # 显示可用层名称、架构选项和其他参数
./script/build_layered.sh --ceres <TAB> # 显示可用层名称和架构选项
./script/build_layered.sh x86 <TAB>     # 显示可用层名称和参数
```

**支持的补全选项**：
- **层名称**: `core_layer`, `hardware_layer`, `perception_layer`, `intelligence_layer`, `application_layer`
- **参数**: `-c` (清理构建), `--ceres` (启用Ceres优化), `help` (帮助信息)
- **架构选项**: `x86` (x86_64架构), `arm` (ARM架构)
- **智能上下文**: 根据已输入的参数智能提示剩余选项

**安装补全功能**：
```bash
# 永久安装补全功能（推荐）
./completions/install_completion.sh install

# 查看安装状态
./completions/install_completion.sh status

# 卸载补全功能
./completions/install_completion.sh uninstall
```

**安装方式**：
- **系统级安装**: 复制到 `/etc/bash_completion.d/` (需要sudo权限，所有用户可用)
- **用户级安装**: 复制到 `~/.local/share/bash-completion/completions/` (仅当前用户)
- **配置文件**: 添加到 `~/.bashrc` (备选方案)

**注意事项**：
- 补全功能支持 `./script/build_layered.sh`、`./build_layered.sh` 等多种调用方式
- **安装后永久生效**，无需每次重新加载
- 首次安装后需要重启终端或执行 `source ~/.bashrc` 生效
- 补全功能会自动过滤已使用的参数，避免重复输入
- 安装脚本会自动检测最适合的安装方式，并避免重复安装

### 构建脚本对比

| 脚本 | 位置 | 用途 | 特点 |
|-----|-----|------|------|
| `build.sh` | 项目根目录 | 快速编译所有层 | **新增**，简化命令，一键编译所有层 |
| `build_layered.sh` | 项目根目录 | 分层编译（外部） | 支持指定层编译和 Ceres 控制 |
| `build_layered_ceres.sh` | 项目根目录 | Ceres 优化编译 | 便捷启用 Ceres 的编译脚本 |
| `script/build_layered.sh` | 容器内部 | 分层编译（内部） | 容器内使用，底层实现，支持 Ceres 参数 |

---

## 🔄 开发工作流

### 新功能开发

```bash
# 1. 更新到最新代码
./sync.sh pull

# 2. 切换到开发分支（可选）
./sync.sh switch devel

# 3. 进入目标层开发
cd src/intelligence_layer
git checkout -b feature/new-navigation

# 4. 开发并提交
# ... 开发代码 ...
git add . && git commit -m "feat: implement new navigation"
git push origin feature/new-navigation

# 5. 返回主目录测试构建
cd ../..
./build_layered.sh intelligence_layer    # 分层构建测试
```

### 集成测试

```bash
# 1. 确保所有层都是最新的
./sync.sh status

# 2. 完整构建
./build.sh                          # 快速构建所有层

# 3. 或者使用分层构建
./build_layered.sh intelligence_layer  # 树莓派/ARM只需编译到智能层
./build_layered.sh application_layer   # Orin/x86编译到应用层

# 4. 启动测试环境（使用新的 system_bringup 包）

# 按平台启动（推荐）
ros2 launch system_bringup qr_raspi.launch.py   # 树莓派（3层）
ros2 launch system_bringup qr_orin.launch.py    # Orin（4层）
ros2 launch system_bringup qr_arm.launch.py     # 通用ARM（3层）

# 或按层级启动
ros2 launch system_bringup 1_hardware.launch.py      # 仅硬件层
ros2 launch system_bringup 3_intelligence.launch.py  # 到智能层
ros2 launch system_bringup 4_application.launch.py   # 完整系统

# 或使用旧的 bringup（逐步废弃）
cd src/application_layer
source setup.bash
ros2 launch bringup system.launch.py
```

### 版本发布

```bash
# 1. 验证发布状态
./release.sh validate

# 2. 创建发布版本
./release.sh create v1.2.0

# 3. 推送主工作空间
git push origin main --tags
```

---

## 🚀 System Bringup - 分层启动系统

### 概述

`system_bringup` 是位于 **core_layer** 的系统级启动包，支持按层级和平台特定的方式启动 ROS2 节点。

### 核心特性

- ✅ **分层启动**: 独立启动任意层级（硬件层、感知层、智能层、应用层）
- ✅ **平台适配**: 针对不同硬件平台（树莓派、Orin、ARM、x86）自动配置
- ✅ **解耦编译**: 无需编译完整5层即可运行系统
- ✅ **灵活配置**: 通过启动参数控制功能模块开关

### 为什么需要 system_bringup？

**问题**: 原来的 `bringup` 包在 `application_layer`（第5层），导致必须编译完整5层才能运行系统

**解决方案**: 将 `system_bringup` 放在 `core_layer`（第1层），支持：
- 树莓派/ARM：只需编译到第3层（智能层）即可运行
- Orin/x86：编译到第4层（应用层）获得完整功能
- 调试模式：只编译第1层（硬件层）快速测试

### 快速使用

#### 1. 按平台启动（推荐）

```bash
# 树莓派平台（3层架构，低功耗）
ros2 launch system_bringup qr_raspi.launch.py

# Orin平台（4层架构，高性能）
ros2 launch system_bringup qr_orin.launch.py

# 通用ARM平台
ros2 launch system_bringup qr_arm.launch.py

# 调试模式（仅硬件层）
ros2 launch system_bringup qr_debug.launch.py
```

#### 2. 按层级启动

```bash
# 启动硬件层（仅运动控制和底盘）
ros2 launch system_bringup 1_hardware.launch.py

# 启动到感知层（硬件 + 相机/音频）
ros2 launch system_bringup 2_perception.launch.py

# 启动到智能层（硬件 + 感知 + 导航/跟随）
ros2 launch system_bringup 3_intelligence.launch.py

# 启动到应用层（完整系统）
ros2 launch system_bringup 4_application.launch.py
```

#### 3. 自定义配置

```bash
# 树莓派上启动，禁用音频模块
ros2 launch system_bringup qr_raspi.launch.py enable_audio:=false

# 智能层启动，启用智能跟随
ros2 launch system_bringup 3_intelligence.launch.py enable_follow:=true

# 硬件层启动，使用仿真时间
ros2 launch system_bringup 1_hardware.launch.py use_sim_time:=true
```

### 平台配置说明

| 平台 | 架构 | 最大层级 | 适用场景 | 控制频率 |
|------|------|---------|---------|---------|
| `qr_raspi` | ARM | 3 (智能层) | 低功耗、基础功能 | 30Hz |
| `qr_orin` | ARM+GPU | 4 (应用层) | 高性能、完整功能 | 50Hz |
| `qr_arm` | ARM | 3 (智能层) | 平衡性能和功能 | 40Hz |
| `qr_debug` | 任意 | 1 (硬件层) | 快速测试、故障排查 | 20Hz |

### 层级依赖关系

```
4_application (应用层) - dev_server, remote_ctrl, key_control
    ↓ 依赖 ↓
3_intelligence (智能层) - navigation, bt_manager, smart_follow, path_tracker
    ↓ 依赖 ↓
2_perception (感知层) - camera_ptz, speaker, tts, speech_recognition
    ↓ 依赖 ↓
1_hardware (硬件层) - motion_control, robot_base
```

### 依赖检查

```bash
# 检查所有层级的依赖
ros2 run system_bringup check_dependencies.py

# 检查特定层级
ros2 run system_bringup check_dependencies.py hardware_layer
ros2 run system_bringup check_dependencies.py intelligence_layer
```

### 与 systemd 集成

修改 `/home/wl/autorun/iiri-ros/start_ros2_iiri_start.sh`：

```bash
#!/bin/bash
source /opt/ros/humble/setup.bash
source /home/wl/autorun/iiri-ros/install/setup.bash

# 根据平台选择启动文件
PLATFORM="qr_raspi"  # 或 qr_orin, qr_arm

# 启动系统
ros2 launch system_bringup ${PLATFORM}.launch.py
```

### 迁移指南

从旧的 `application_layer/bringup` 迁移：

1. **编译新包**:
   ```bash
   ./build_layered.sh intelligence_layer  # 树莓派/ARM
   # 或
   ./build_layered.sh application_layer   # Orin/x86
   ```

2. **更新启动命令**:
   ```bash
   # 旧方式
   ros2 launch bringup qr_raspi.launch.py

   # 新方式
   ros2 launch system_bringup qr_raspi.launch.py
   ```

3. **更新 systemd 服务**: 修改启动脚本中的包名和launch文件

详细文档请查看: `src/core_layer/src/system_bringup/README.md`

---

## 🌟 核心优势

### 对比传统单仓库

| 方面 | 传统单仓库 | vcstool分层架构 |
|------|------------|-----------------|
| **代码组织** | 单一大仓库，混乱 | 清晰的分层模块化 |
| **团队协作** | 频繁冲突，效率低 | 独立并行开发 |
| **版本控制** | 整体版本，粗粒度 | 分层精确版本控制 |
| **构建效率** | 全量构建，时间长 | 增量构建，速度快 |
| **依赖管理** | 隐式依赖，难追踪 | 显式声明，清晰明了 |
| **发布管理** | 整体发布，风险大 | 分层发布，风险可控 |

### 主要特性

- ✨ **分层解耦**：清晰的架构边界，便于维护
- 🚀 **并行开发**：多团队可同时开发不同层
- 🎯 **精确控制**：每层独立版本，精确管理
- ⚡ **高效构建**：只构建修改的层，大幅提升效率
- 📊 **依赖透明**：通过vcstool明确依赖关系
- 🔄 **灵活切换**：轻松在不同版本间切换

---

## 🚦 环境要求

### 系统要求

- **操作系统**: Ubuntu 20.04/22.04
- **ROS版本**: ROS2 Humble
- **Python版本**: Python 3.8+

### 网络要求

- 能够访问内网GitLab服务器：`192.168.1.55`
- 能够访问ROS2官方软件源

### 硬件要求

- **最低配置**: 4GB RAM, 20GB 硬盘空间
- **推荐配置**: 8GB RAM, 50GB 硬盘空间

---

## 🛠️ 故障排除

### 常见问题

1. **vcstool导入失败**
   ```bash
   # 检查网络连接
   ping 192.168.1.55

   # 重新配置Git凭据
   git config --global credential.helper store
   ```

2. **构建失败**
   ```bash
   # 安装缺失依赖（Docker镜像内已包含）
   rosdep install --from-paths src --ignore-src -r -y

   # 清理重新构建
   ./build.sh -c                    # 清理后重新构建所有层
   ./build_layered.sh -c            # 或者清理后分层构建
   ```

3. **权限问题**
   ```bash
   # 检查Git配置
   git config --list | grep user

   # 重新设置权限
   ./install.sh --help
   ```

### 获取支持

- 📖 查看[团队操作指南](./VCSTOOL_TEAM_GUIDE.md)
- 💬 在团队群里提问
- 🔧 联系技术负责人
- 📝 在GitLab创建Issue

---

## 📈 项目统计

### 仓库信息

- **主工作空间**: `iiri_ros2_architecture`
- **分层仓库数**: 5个
- **总包数**: ~50个
- **支持架构**: x86_64, aarch64

### 版本信息

- **当前版本**: v1.0.0
- **ROS2版本**: Humble
- **vcstool版本**: v0.3.0

---

## 📄 许可证

本项目为私有项目，版权归IIRI团队所有。

---

## 🤝 贡献指南

1. Fork项目
2. 创建功能分支 (`git checkout -b feature/AmazingFeature`)
3. 提交更改 (`git commit -m 'Add some AmazingFeature'`)
4. 推送到分支 (`git push origin feature/AmazingFeature`)
5. 创建Pull Request

---

**📝 文档版本**: v2.0.0 | **📅 最后更新**: 2024-09-28
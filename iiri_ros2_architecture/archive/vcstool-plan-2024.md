# ROS2分层架构代码管理方案

> **⚠️ 历史归档文档**  
> **原文档位置**: `docs/vsctool_plan`  
> **归档时间**: 2025-10-13  
> **状态**: 已实现并废弃，仅供历史参考  
> **说明**: 本文档是 2024 年早期的 vcstool 规划文档，现在项目已完全实现该方案。当前文档请参考 [vcstool 使用指南](../docs/development/vcstool-guide.md)

## 1. 方案概述

### 1.1 核心思路
采用 **单个Docker镜像** + **vcstool** + **分层Git仓库** 的混合管理方案，既保持了环境的统一性，又实现了代码的模块化管理。

### 1.2 设计原则
- **单一镜像**：保持现有的Docker镜像优势，避免多镜像复杂性
- **分层解耦**：每层独立Git仓库，支持并行开发
- **依赖清晰**：通过vcstool明确层间依赖关系
- **版本控制**：支持层级版本管理和回滚
- **开发友好**：保持现有开发习惯，降低迁移成本

## 2. 架构设计

### 2.1 仓库结构
```
iiri_ros2_workspace/              # 主工作空间仓库（vcstool配置）
├── .repos                        # vcstool配置文件
├── README.md                     # 主文档
├── build.sh                      # 统一构建脚本
├── docker.sh                     # Docker环境脚本
└── docs/                         # 文档目录

src/                              # 源码目录（由vcstool管理）
├── core_layer/                   # 核心层仓库
│   ├── interface/
│   └── third_party/
├── hardware_layer/               # 硬件层仓库
│   ├── motion_control/
│   ├── robot_base/
│   └── sensor/
├── perception_layer/             # 感知层仓库
│   ├── camera_ptz/
│   ├── speaker/
│   ├── speech_recognition/
│   └── tts/
├── intelligence_layer/           # 智能层仓库
│   ├── navigation/
│   ├── bt_manager/
│   ├── smart_follow/
│   └── xiaozhi/
└── application_layer/            # 应用层仓库
    ├── bringup/
    ├── dev_server/
    ├── key_control/
    ├── record/
    └── remote_ctrl/
```

### 2.2 Git仓库划分
| 仓库名称 | 用途 | 包含包 |
|---------|------|--------|
| `iiri-ros2-workspace` | 主工作空间，vcstool配置 | 构建脚本、文档、配置 |
| `iiri-core-layer` | 核心基础层 | interface, third_party |
| `iiri-hardware-layer` | 硬件抽象层 | motion_control, robot_base, sensor |
| `iiri-perception-layer` | 感知处理层 | camera_ptz, speaker, speech_recognition, tts |
| `iiri-intelligence-layer` | 智能导航层 | navigation, bt_manager, smart_follow, xiaozhi |
| `iiri-application-layer` | 应用通信层 | bringup, dev_server, key_control, record, remote_ctrl |

## 3. vcstool配置

### 3.1 .repos文件结构
```yaml
repositories:
  src/core_layer:
    type: git
    url: http://192.168.1.55/ontology/iiri-core-layer.git
    version: main
  src/hardware_layer:
    type: git
    url: http://192.168.1.55/ontology/iiri-hardware-layer.git
    version: main
  src/perception_layer:
    type: git
    url: http://192.168.1.55/ontology/iiri-perception-layer.git
    version: main
  src/intelligence_layer:
    type: git
    url: http://192.168.1.55/ontology/iiri-intelligence-layer.git
    version: main
  src/application_layer:
    type: git
    url: http://192.168.1.55/ontology/iiri-application-layer.git
    version: main
```

### 3.2 版本管理策略
```yaml
# 开发版本配置 (.repos.devel)
repositories:
  src/core_layer:
    type: git
    url: http://192.168.1.55/ontology/iiri-core-layer.git
    version: devel
  # ... 其他层使用devel分支

# 稳定版本配置 (.repos.stable)
repositories:
  src/core_layer:
    type: git
    url: http://192.168.1.55/ontology/iiri-core-layer.git
    version: v1.0.0
  # ... 其他层使用对应稳定版本标签
```

## 4. Docker集成

### 4.1 保持现有镜像
- **x86**: `192.168.1.93/iiri/build_x86_ros2:v1.4.3`
- **ARM**: `192.168.1.93/iiri/build_arm_ros2:v1.4.2`

### 4.2 优化docker.sh脚本
```bash
#!/bin/bash
# 检测架构并启动对应Docker镜像
# 自动挂载工作空间
# 预安装vcstool
```

### 4.3 容器内环境
```bash
# 预装软件
- vcstool
- colcon
- rosdep

# 预设环境变量
- ROS_DOMAIN_ID
- CMAKE_PREFIX_PATH
- 架构相关变量
```

## 5. 开发工作流

### 5.1 项目初始化
```bash
# 1. 克隆主工作空间
git clone http://192.168.1.55/ontology/iiri-ros2-workspace.git
cd iiri-ros2-workspace

# 2. 导入所有层代码
vcs import src < .repos

# 3. 启动Docker环境
./docker.sh

# 4. 容器内构建
./build.sh
```

### 5.2 日常开发流程
```bash
# 1. 更新代码
vcs pull src                      # 更新所有层
vcs pull src --only=core_layer    # 只更新指定层

# 2. 分层开发
cd src/intelligence_layer         # 进入目标层
git checkout -b feature/new-nav   # 创建功能分支
# ... 开发代码

# 3. 测试构建
./build_layered.sh intelligence_layer

# 4. 提交代码
cd src/intelligence_layer
git add . && git commit -m "feat: add new navigation feature"
git push origin feature/new-nav

# 5. 更新主工作空间版本
cd ../../  # 回到主工作空间
# 更新.repos文件中的版本信息
git add .repos && git commit -m "update: intelligence_layer to feature/new-nav"
```

### 5.3 版本发布流程
```bash
# 1. 各层创建发布标签
cd src/core_layer && git tag v1.0.1
cd ../hardware_layer && git tag v1.0.1
# ... 为所有层打标签

# 2. 更新稳定版本配置
cp .repos.stable .repos
# 修改.repos中的version为对应标签

# 3. 创建主工作空间发布
git add .repos
git commit -m "release: v1.0.1"
git tag v1.0.1
git push origin main --tags
```

## 6. CI/CD集成

### 6.1 单层CI流程
```yaml
# .github/workflows/layer-ci.yml
name: Layer CI
on: [push, pull_request]
jobs:
  test:
    runs-on: ubuntu-latest
    container: 192.168.1.93/iiri/build_x86_ros2:v1.4.3
    steps:
      - uses: actions/checkout@v2
      - name: Install dependencies
        run: rosdep install --from-paths . --ignore-src -r -y
      - name: Build layer
        run: colcon build
      - name: Run tests
        run: colcon test
```

### 6.2 集成CI流程
```yaml
# 主工作空间CI
name: Integration CI
on: [push, pull_request]
jobs:
  integration-test:
    runs-on: ubuntu-latest
    container: 192.168.1.93/iiri/build_x86_ros2:v1.4.3
    steps:
      - uses: actions/checkout@v2
      - name: Import repositories
        run: vcs import src < .repos
      - name: Build all layers
        run: ./build.sh
      - name: Run integration tests
        run: ./test_integration.sh
```

## 7. 迁移指南

### 7.1 迁移步骤
1. **创建新仓库**
   ```bash
   # 在GitLab创建6个新仓库
   # iiri-ros2-workspace (主)
   # iiri-core-layer
   # iiri-hardware-layer
   # iiri-perception-layer
   # iiri-intelligence-layer
   # iiri-application-layer
   ```

2. **分层代码拆分**
   ```bash
   # 提取每层代码到独立仓库
   git subtree push --prefix=core_layer origin core-layer-branch
   # 对每层重复此操作
   ```

3. **配置vcstool**
   ```bash
   # 在主仓库创建.repos文件
   # 配置所有层的Git URL和版本
   ```

4. **验证完整性**
   ```bash
   # 测试vcs import
   # 验证构建流程
   # 确认所有功能正常
   ```

### 7.2 迁移注意事项
- **备份**：迁移前完整备份现有代码
- **渐进式**：可先保持现有结构，并行测试新方案
- **团队培训**：提前培训团队vcstool使用
- **文档更新**：及时更新所有相关文档

## 8. 管理工具

### 8.1 常用vcstool命令
```bash
# 导入所有仓库
vcs import src < .repos

# 检查状态
vcs status src

# 拉取更新
vcs pull src

# 切换分支
vcs custom src --args checkout devel

# 显示版本信息
vcs log src --limit 1
```

### 8.2 自定义脚本
```bash
# sync.sh - 同步脚本
#!/bin/bash
echo "正在同步所有层..."
vcs pull src
echo "检查状态..."
vcs status src

# release.sh - 发布脚本
#!/bin/bash
echo "创建发布版本..."
# 自动化版本标签和发布流程
```

## 9. 优势分析

### 9.1 对比现有方案
| 方面 | 当前单仓库 | 新方案 |
|------|------------|--------|
| 代码组织 | 单一大仓库 | 分层模块化 |
| 开发协作 | 容易冲突 | 独立并行开发 |
| 版本控制 | 整体版本 | 分层版本管理 |
| 构建效率 | 全量构建 | 增量构建 |
| 依赖管理 | 隐式依赖 | 显式依赖声明 |

### 9.2 核心优势
- **模块化**：清晰的层级边界，便于维护
- **并行开发**：多团队可同时开发不同层
- **版本控制**：每层独立版本，精确控制
- **增量构建**：只构建修改的层，提升效率
- **依赖透明**：通过vcstool明确依赖关系

## 10. 最佳实践

### 10.1 分支管理
- **main分支**：稳定发布版本
- **devel分支**：开发集成分支
- **feature分支**：功能开发分支
- **hotfix分支**：紧急修复分支

### 10.2 版本规范
- **语义化版本**：主版本.次版本.修订版本
- **标签命名**：v1.0.0, v1.0.1
- **发布说明**：详细的CHANGELOG

### 10.3 开发规范
- **层级原则**：严格遵循上层依赖下层
- **接口稳定**：下层接口变更需要版本升级
- **向后兼容**：次版本更新保持向后兼容
- **测试覆盖**：每层都要有完整测试

## 11. 故障排除

### 11.1 常见问题
```bash
# vcs import失败
# 检查网络连接和仓库权限
vcs import src < .repos --debug

# 依赖冲突
# 检查各层版本兼容性
rosdep check --from-paths src --ignore-src

# 构建失败
# 检查层级依赖顺序
./build_layered.sh --debug
```

### 11.2 回滚方案
```bash
# 快速回滚到稳定版本
cp .repos.stable .repos
vcs import src < .repos --force
./build.sh
```

这个方案充分利用了vcstool的ROS2生态兼容性，同时保持了Docker的环境一致性，为你们的分层架构提供了一个完整的代码管理解决方案。
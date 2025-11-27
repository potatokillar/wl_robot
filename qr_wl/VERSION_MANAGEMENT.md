# 版本管理系统说明

## 📋 概述

本项目采用基于Git分支的自动版本管理系统，根据当前分支自动确定软件版本号：

- **开发分支**：`software version = "分支名_编译时间"`
- **主分支**：`software version = "远程最新tag版本"`

## 🚀 版本规则

### 开发模式（非main分支）
- **分支**：`dev`、`feature/xxx`、`bugfix/xxx` 等任何非main分支
- **版本格式**：`分支名_YYYYMMDD_HHMMSS`
- **示例**：`dev_20250925_093000`、`feature/login_20250925_143000`

### 发布模式（main分支）
- **分支**：`main`
- **版本格式**：远程仓库最新git tag
- **示例**：`v2.0.0`、`v1.5.3`

## 📖 使用指南

### 1. 日常开发流程

```bash
# 切换到开发分支
git checkout dev
# 或创建新的功能分支
git checkout -b feature/new-feature

# 编译项目
./build.sh

# 查看版本信息
./release.sh
```

**输出示例**：
```
=== 版本状态 ===
当前分支: dev
最新commit: a1c8fb3
远程最新tag: v1.0.0
版本模式: 开发模式 (dev_20250925_093000)
```

### 2. 发布版本流程

```bashit info
# 切换到主分支
git checkout main

# 拉取最新代码
git pull origin main

# 编译发布版本（自动获取远程最新tag）
./build.sh

# 查看版本状态
./release.sh
```

**输出示例**：
```
=== 版本状态 ===
当前分支: main
最新commit: a1c8fb3
远程最新tag: v1.0.0
版本模式: 发布模式 (使用远程最新tag: v2.0.0)
```

## 🔍 版本信息查看

### 1. 运行时版本信息

程序启动时会输出版本信息：

```bash
# 运行编译好的程序
cd build/x64/output
./qr config.toml

# 输出示例：
# 开发版本：
[10:14:09-671][info][robot] robot type:config.toml, software version:dev_20250925_093000, sdk version:v1.16.0
[10:14:09-671][info][robot] git info: dev_a1c8fb3_48e2

# 发布版本：
[10:14:09-671][info][robot] robot type:config.toml, software version:v2.0.0, sdk version:v1.16.0
[10:14:09-671][info][robot] it info: main_a1c8fb3_48e2
```

### 2. Git信息解析

**Git info格式**：`分支名_commit短号_程序MD5后5位`

- **分支名**：当前Git分支名称
- **commit短号**：当前commit的短hash（7位）
- **程序MD5后5位**：编译生成的程序文件MD5值的最后5位

**示例**：`main_a1c8fb3_48e2`
- `main`：主分支
- `a1c8fb3`：commit短号
- `48e2`：程序MD5的最后5位

## 🛠️ 脚本说明

### release.sh - 版本状态查看脚本

```bash
# 查看当前版本状态
./release.sh

# 显示帮助信息
./release.sh help
```

**功能**：
- 显示当前Git分支
- 显示最新commit信息
- 获取并显示远程最新tag
- 显示当前版本模式和预期版本号

### build.sh - 编译脚本

```bash
# 编译所有架构
./build.sh

# 只编译x86版本
./build.sh x86

# 只编译arm版本
./build.sh arm

# 清理并编译
./build.sh -c
```

**版本处理**：
- 编译时自动检测当前Git分支
- 根据分支规则生成相应的软件版本号
- 在main分支时自动获取远程最新tag

## 📁 相关文件

```
qr_wl/
├── VERSION_MANAGEMENT.md          # 版本管理说明文档（本文件）
├── release.sh                     # 版本状态查看脚本
├── build.sh                       # 编译脚本
├── CMakeLists.txt                 # 构建配置（包含版本逻辑）
├── app/config/git_info.hpp.in     # Git信息模板文件
├── app/config/device/config_base.hpp # 版本配置基类
└── project/real/main.cpp          # 主程序（输出版本信息）
```

## 🔧 技术实现

### CMake配置逻辑

1. **分支检测**：使用`git rev-parse --abbrev-ref HEAD`获取当前分支
2. **远程tag获取**：执行`git fetch --tags`和`git tag --sort=-version:refname`获取最新tag
3. **版本生成**：
   - main分支：`SOFTWARE_VERSION = 最新tag`
   - 其他分支：`SOFTWARE_VERSION = 分支名_时间戳`
4. **头文件生成**：生成`git_info.hpp`包含所有版本信息

### 程序运行时

1. **版本显示**：`PrintRequiredInfo()`函数输出软件版本
2. **Git信息**：实时计算程序文件MD5并组合git信息
3. **MD5计算**：使用`/proc/self/exe`获取程序路径并计算MD5后5位

## 🚨 注意事项

### 编译环境要求

- Git环境正常配置
- 能够访问远程Git仓库
- 具有md5sum命令（用于计算程序MD5）

### 版本号规范

- **Git tag格式**：必须遵循`vX.Y.Z`格式（如：v2.0.0）
- **分支命名**：建议使用有意义的分支名（如：dev、feature/login、bugfix/auth）

### 常见问题

1. **网络问题**：如果无法连接远程仓库，main分支将使用默认版本`v1.0.0`
2. **无tag情况**：如果远程仓库没有tag，将使用默认版本`v1.0.0`
3. **权限问题**：确保对Git仓库有读取权限

## 💡 最佳实践

1. **开发阶段**：在feature分支或dev分支进行开发，便于版本区分
2. **发布前测试**：在main分支编译前确保本地代码已提交
3. **版本追踪**：通过git info信息可精确追踪每个构建的代码版本
4. **CI/CD集成**：脚本支持Docker环境，适合CI/CD流水线使用

## 📞 联系方式

如有问题或建议，请通过以下方式联系：

- 提交Issue到项目仓库
- 联系项目维护人员

---

**版本管理系统版本**: v1.0
**最后更新**: 2025-09-25
**文档维护**: 项目开发团队
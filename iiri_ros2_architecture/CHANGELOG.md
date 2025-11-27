# Changelog

本文档记录 IIRI ROS2 分层架构项目的所有重要变更。

格式基于 [Keep a Changelog](https://keepachangelog.com/zh-CN/1.0.0/)，
版本号遵循 [语义化版本](https://semver.org/lang/zh-CN/)。

## [Unreleased]

### 计划中
- 快速开始指南（getting-started/）
- API 参考文档
- 故障排除手册

## [2.1.0] - 2025-10-13

### Added 新增
- **文档重组**：完整重构 `docs/` 目录结构
  - 新增 7 个主题目录：getting-started, architecture, development, deployment, ci-cd, testing, reference
  - 创建文档导航中心 `docs/README.md`
  - 添加文档元信息（位置、更新时间、作者）
- **部署指南**：新增完整的部署文档
  - `docs/deployment/deployment-guide.md` - 从编译到生产的完整流程
  - 包含版本管理和回滚策略
  - 故障排除和最佳实践
- **测试文档归档**：创建测试报告归档机制
  - `docs/testing/test-reports/` - 历史测试报告目录
  - 测试报告统一命名：日期+测试名称
- **历史归档**：创建 `archive/` 目录存放过时文档
  - 归档 vcstool 早期规划文档

### Changed 变更
- **文档命名规范化**：统一使用小写+连字符命名
  - ~~`DEPLOYMENT_FLOW.md`~~ → `deployment/deployment-guide.md`
  - ~~`JENKINS_CI_SETUP.md`~~ → `ci-cd/jenkins-setup.md`
  - ~~`TEST_BRINGUP.md`~~ → `testing/testing-guide.md`
  - ~~`README_PATH_TRACKER.md`~~ → `architecture/path-tracker.md`
- **CLAUDE.md 更新**：新增文档结构章节
  - 添加完整的文档导航说明
  - 更新文档链接到新位置
- **署名统一**：所有文档作者统一为"唐文浩"

### Removed 移除
- 删除 `docs/` 根目录下的旧文档文件
  - 旧文件已迁移到分类目录或归档

## [2.0.0] - 2025-10-11

### Added 新增
- **System Bringup 重构**：将 bringup 从 application_layer 移至 core_layer
  - 减少编译依赖：从 5 层降至 3 层
  - 支持平台特定配置（pi, orin）
  - 创建完整测试套件
- **CI/CD 增强**：Jenkins 自动构建和部署
  - 配置自动化构建 Pipeline
  - 实现构建产物归档
  - 优化并行编译策略
  - 添加版本追踪（Git commit hash）
- **部署改进**：智能符号链接管理
  - 版本特定部署路径
  - 快速回滚能力
  - 自动化服务部署脚本
- **开发工具集成**：
  - Docker MCP - 容器管理
  - SSH MCP - 树莓派远程调试（192.168.1.54）
  - Jenkins MCP - CI/CD 监控
  - Python 3.10/3.12 并存环境
- **MCP 服务器配置**：
  - Jenkins MCP 用于 CI/CD 集成
  - Docker MCP 用于容器管理
  - SSH MCP 用于远程设备调试

### Changed 变更
- **署名规范化**：
  - 统一所有文档和代码署名为"唐文浩"
  - 更新 Git commit 格式
  - 清理所有 "Claude Code" 引用
- **构建优化**：
  - Jenkins 并行编译数降至 1（避免内存耗尽）
  - 自动触发频率从 5 分钟改为 2 小时
- **文档更新**：
  - 新增 `TEST_RESULTS.md` - System Bringup 测试报告
  - 新增 `DEPLOYMENT_FLOW.md` - 部署流程说明
  - 更新所有相关指南和配置说明

### Fixed 修复
- Docker-in-Docker 路径映射问题
- Docker 容器命名冲突
- Jenkins 归档步骤执行顺序
- Launch 文件条件判断（IfCondition）

## [1.0.0] - 2024-09-28

### Added 新增
- **五层架构实现**：
  - core_layer - 核心基础层
  - hardware_layer - 硬件抽象层
  - perception_layer - 感知处理层
  - intelligence_layer - 智能导航层
  - application_layer - 应用通信层
- **vcstool 管理**：多仓库代码管理
  - `.repos` 配置文件
  - `sync.sh` 同步脚本
  - 支持多分支切换
- **分层构建系统**：
  - `build_layered.sh` - 主构建脚本
  - `build.sh` - 快速构建脚本
  - 支持 x86 和 ARM 架构
  - 可选 Ceres 优化
- **Path Tracker**：双模式路径跟踪
  - 基础模式（几何计算）
  - 优化模式（Ceres 非线性优化）
- **Docker 环境**：
  - x86 镜像: `192.168.1.93/iiri/build_x86_ros2:v1.4.3`
  - ARM 镜像: `192.168.1.93/iiri/build_arm_ros2:v1.4.2`
- **基础文档**：
  - `README.md` - 项目总览
  - `CLAUDE.md` - AI 助手指南
  - `VCSTOOL_GUIDE.md` - vcstool 详细文档
  - `VCSTOOL_TEAM_GUIDE.md` - 团队协作指南

### Changed 变更
- 从单一仓库迁移到多仓库架构
- 重构编译系统支持分层构建

---

**维护者**: 唐文浩
**最后更新**: 2025-10-13

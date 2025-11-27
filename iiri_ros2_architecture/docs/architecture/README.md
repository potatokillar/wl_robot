# 架构文档导航

**作者**: 唐文浩
**最后更新**: 2025-10-30
**版本**: v2.0.0

---

## 📖 文档概览

本目录包含 IIRI 机器人系统的完整架构文档,涵盖新架构 (ROS2 + ros2_control) 和旧架构 (qr_wl/wl_ros) 的对比分析。

---

## 📚 主要文档

### 1. [超详细架构图 (推荐阅读)](./DETAILED_ARCHITECTURE_DIAGRAMS.md)

**文件**: `DETAILED_ARCHITECTURE_DIAGRAMS.md`
**版本**: v2.0.0
**创建日期**: 2025-10-30

#### 内容概要

这是最完整、最详细的架构图文档,包含以下10个章节:

1. **系统整体拓扑图** - 物理设备、网络连接、端口映射
2. **通信流程与协议详解** - JSON-RPC 2.0, Binary Protocol, WebSocket 详细格式
3. **ROS2 五层架构详细组件图** - 13个节点、Topic/Service 通信矩阵
4. **qr_wl 详细架构** - 旧架构参考,15000行单体程序
5. **wl_ros 详细架构** - ROS1 集成方案
6. **ros2_control 新架构详细图** - 1000Hz 控制,模块化设计
7. **qr_wl → ros2_control 迁移对比** - 三代架构演进,性能提升 1291% ROI
8. **OTA 升级完整流程** - 状态机、组件架构、时序图
9. **qr_chart 监控客户端详细架构** - Qt6 四层架构、实时数据流
10. **网络拓扑与数据流** - 完整的网络连接和数据流向

#### 特色亮点

- ✅ **所有端口号**: 20333, 20334, 43000, 8080, 9000 等
- ✅ **所有协议格式**: JSON-RPC 2.0 示例、Binary Protocol 帧结构
- ✅ **所有 ROS2 Topics**: /run_state, /cmd_vel, /joint_states 等
- ✅ **性能对比表**: qr_wl vs wl_ros vs ros2_control 全方位对比
- ✅ **Mermaid 图表**: 30+ 张高质量架构图

#### 适用人群

- 🎯 新加入团队的开发者(快速了解系统全貌)
- 🎯 需要详细技术细节的架构师
- 🎯 需要对比新旧架构的决策者
- 🎯 需要端口和协议信息的运维人员

---

### 2. [系统架构文档](./SYSTEM_ARCHITECTURE.md)

**文件**: `SYSTEM_ARCHITECTURE.md`
**版本**: v1.0.0
**创建日期**: 2025-10-30

#### 内容概要

文字为主的系统架构说明文档,包含:

1. **系统概述** - 总体设计理念
2. **ROS2 五层架构** - 层级依赖关系
3. **qr_wl → ros2_control 迁移方案** - 迁移动机、收益分析
4. **qr_chart 监控客户端** - Qt6 架构设计
5. **OTA 升级系统** - update_manager 设计
6. **CI/CD 流程** - Jenkins 自动化
7. **部署架构** - systemd 服务管理
8. **技术栈** - 完整的技术选型
9. **关键决策记录** - 架构决策文档 (ADR)

#### 特色亮点

- ✅ **文字描述详细**: 适合阅读理解
- ✅ **架构决策记录**: 为什么这样设计
- ✅ **技术栈完整**: 所有依赖库和工具
- ✅ **部署流程**: 从构建到上线的完整步骤

#### 适用人群

- 🎯 需要理解设计理念的产品经理
- 🎯 需要评估技术方案的技术负责人
- 🎯 需要了解部署流程的运维工程师

---

### 3. [架构图集](./ARCHITECTURE_DIAGRAMS.md)

**文件**: `ARCHITECTURE_DIAGRAMS.md`
**版本**: v1.0.0
**创建日期**: 2025-10-30

#### 内容概要

早期版本的架构图文档,包含8个章节:

1. 系统整体部署架构
2. ROS2 五层架构详细图
3. qr_wl → ros2_control 迁移架构
4. 网络通信和数据流
5. OTA 升级流程
6. qr_chart 监控客户端架构
7. CI/CD 流水线
8. 完整技术栈

#### 特色亮点

- ✅ **图表清晰**: Mermaid 格式,易于维护
- ✅ **覆盖全面**: 系统各个方面都有覆盖

#### 适用人群

- 🎯 需要快速预览的读者
- 🎯 已经熟悉系统,只需查看图表的开发者

---

## 📊 旧架构参考资料

### 旧架构图 (PNG 格式)

位置: `/home/wl/twh/workspace/ontology/architecture_diagrams/`

| 文件名 | 内容 | 用途 |
|--------|------|------|
| `communication_flow.png` | 通信流程与协议详解 | 端口、协议、数据格式参考 |
| `qr_wl_detailed.png` | qr_wl 详细架构 (15000行) | 单体程序架构参考 |
| `robot_system_architecture.png` | 机器人系统总体架构 | 系统分层参考 |
| `wl_ros_detailed.png` | wl_ros 详细架构 (ROS1) | ROS1 集成方案参考 |

**注意**: 这些是旧架构的图表,仅供参考对比使用。新架构请参阅 `DETAILED_ARCHITECTURE_DIAGRAMS.md`。

---

## 🚀 快速导航

### 根据需求选择文档

| 需求 | 推荐文档 | 章节 |
|------|----------|------|
| **了解系统全貌** | DETAILED_ARCHITECTURE_DIAGRAMS.md | §1, §3, §10 |
| **查看端口和协议** | DETAILED_ARCHITECTURE_DIAGRAMS.md | §2 |
| **理解 ROS2 架构** | DETAILED_ARCHITECTURE_DIAGRAMS.md | §3 |
| **了解旧架构** | DETAILED_ARCHITECTURE_DIAGRAMS.md | §4, §5 |
| **评估迁移方案** | DETAILED_ARCHITECTURE_DIAGRAMS.md | §7 |
| **了解 OTA 升级** | DETAILED_ARCHITECTURE_DIAGRAMS.md | §8 |
| **了解监控客户端** | DETAILED_ARCHITECTURE_DIAGRAMS.md | §9 |
| **阅读设计理念** | SYSTEM_ARCHITECTURE.md | §1, §9 |
| **查看技术栈** | SYSTEM_ARCHITECTURE.md | §8 |
| **了解部署流程** | SYSTEM_ARCHITECTURE.md | §7 |

---

## 📐 架构图类型说明

### Mermaid 图表类型

本文档使用的 Mermaid 图表类型:

| 类型 | 用途 | 示例 |
|------|------|------|
| **graph TB/LR** | 系统拓扑、数据流 | 物理部署图、网络拓扑图 |
| **stateDiagram-v2** | 状态机流程 | OTA 升级状态机 |
| **sequenceDiagram** | 时序交互 | OTA 升级时序图、数据流管道 |
| **classDiagram** | 数据结构 | qr_chart 数据结构 |
| **erDiagram** | 数据库设计 | SQLite Schema |

### 如何渲染 Mermaid 图表

#### 方法1: GitHub/GitLab (推荐)

直接在 GitHub/GitLab 上查看 Markdown 文件,自动渲染。

#### 方法2: VS Code

安装插件: [Markdown Preview Mermaid Support](https://marketplace.visualstudio.com/items?itemName=bierner.markdown-mermaid)

#### 方法3: 在线工具

- [Mermaid Live Editor](https://mermaid.live/)
- [draw.io](https://draw.io/) (支持导入 Mermaid)

#### 方法4: 命令行工具

```bash
# 安装 mermaid-cli
npm install -g @mermaid-js/mermaid-cli

# 生成 PNG 图片
mmdc -i DETAILED_ARCHITECTURE_DIAGRAMS.md -o output.png
```

---

## 🔗 相关文档链接

### 项目根目录文档

- [项目 README](../../README.md) - 项目概览和快速开始
- [CLAUDE.md](../../CLAUDE.md) - AI 辅助开发指南
- [CHANGELOG.md](../../CHANGELOG.md) - 版本历史

### 开发文档

- [构建系统](../development/build-system.md)
- [vcstool 指南](../development/vcstool-guide.md)
- [部署指南](../deployment/deployment-guide.md)

### CI/CD 文档

- [Jenkins 设置](../ci-cd/jenkins-setup.md)
- [Harbor 镜像仓库](../infrastructure/harbor-registry.md)

### 迁移方案

- [qr_wl 迁移计划](/home/wl/twh/workspace/ros2_control/QR_WL_MIGRATION_PLAN.md)
- [Executive Summary (CN)](/home/wl/twh/workspace/ros2_control/EXECUTIVE_SUMMARY_CN.md)

---

## 📝 文档维护指南

### 更新文档时的注意事项

1. **版本号管理**: 使用语义化版本 (v主版本.次版本.修订号)
2. **日期更新**: 每次修改更新 "最后更新" 日期
3. **作者署名**: 所有新文档必须署名 "唐文浩"
4. **Mermaid 格式**: 确保图表语法正确,可在 [Mermaid Live](https://mermaid.live/) 预览
5. **交叉引用**: 更新相关文档的链接

### 文档审核清单

- [ ] 版本号已更新
- [ ] 日期已更新
- [ ] 作者署名正确
- [ ] Mermaid 图表渲染正常
- [ ] 所有链接有效
- [ ] 代码示例可运行
- [ ] 表格对齐整齐
- [ ] 中英文排版规范

---

## 📞 联系方式

如有文档相关问题,请联系:

**作者**: 唐文浩
**邮箱**: [待补充]
**团队**: 西湖大学智能产业研究院 (Westlake IIRI)

---

**最后更新**: 2025-10-30
**文档版本**: v2.0.0
**维护者**: 唐文浩

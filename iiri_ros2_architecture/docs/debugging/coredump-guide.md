# Coredump 分析指南

**作者**: 唐文浩
**日期**: 2025-10-21
**版本**: v1.0.0

本文档详细说明如何使用 IIRI ROS2 系统的 coredump 分析功能来调试崩溃问题。

---

## 📋 目录

1. [系统概述](#系统概述)
2. [配置说明](#配置说明)
3. [使用方法](#使用方法)
4. [分析技巧](#分析技巧)
5. [常见问题](#常见问题)
6. [最佳实践](#最佳实践)

---

## 系统概述

### 功能特性

IIRI ROS2 系统已集成完整的 coredump 分析能力：

| 功能 | 说明 | 状态 |
|------|------|------|
| **系统级 coredump** | 内核自动生成 core 文件 | ✅ 已启用 |
| **systemd-coredump** | 自动管理和压缩 coredump | ✅ 已安装 |
| **backward_ros** | 崩溃时自动打印堆栈 | ✅ 已集成 |
| **调试符号** | RelWithDebInfo 编译模式 | ✅ 已配置 |
| **分析工具** | 自动化分析脚本 | ✅ 已提供 |

### 技术架构

```
┌─────────────────────────────────────────┐
│  ROS2 Node 崩溃 (SIGSEGV, SIGABRT, etc) │
└─────────────────┬───────────────────────┘
                  │
        ┌─────────┴──────────┐
        │                    │
        ↓                    ↓
┌───────────────┐   ┌────────────────────┐
│ backward_ros  │   │ Kernel Coredump    │
│ 打印堆栈到     │   │ 生成 core 文件到   │
│ stderr.log    │   │ /var/coredumps/    │
└───────────────┘   └──────┬─────────────┘
                           │
                           ↓
                  ┌────────────────────┐
                  │ systemd-coredump   │
                  │ 自动压缩和管理      │
                  └──────┬─────────────┘
                         │
                         ↓
                  ┌────────────────────┐
                  │ coredumpctl        │
                  │ 查看和提取工具      │
                  └────────────────────┘
```

---

## 配置说明

### 1. 系统配置

#### 核心限制 (`/etc/security/limits.conf`)

```bash
# Coredump configuration for IIRI ROS2 debugging
*    soft    core    unlimited
*    hard    core    unlimited
root soft    core    unlimited
root hard    core    unlimited
```

#### 内核配置 (`/etc/sysctl.d/99-coredump.conf`)

```bash
# Coredump configuration for IIRI ROS2
kernel.core_pattern=/var/coredumps/core-%e-%p-%t
kernel.core_uses_pid=1
```

参数说明：
- `%e`: 可执行文件名
- `%p`: 进程 PID
- `%t`: Unix 时间戳

示例文件名: `core-motion_control_node-12345-1729483200`

#### systemd-coredump 配置 (`/etc/systemd/coredump.conf.d/custom.conf`)

```ini
[Coredump]
Storage=external       # 保存到磁盘
Compress=yes          # 自动压缩
ProcessSizeMax=2G     # 单个文件最大 2GB
ExternalSizeMax=10G   # 总计最多 10GB
MaxUse=20G           # 磁盘使用上限
KeepFree=5G          # 保留空闲空间
```

### 2. 服务配置

#### Systemd 服务 (`iiri-ros/iiri-ros.service`)

```ini
[Service]
# Coredump configuration for debugging
LimitCORE=infinity    # 允许生成 coredump
LimitNOFILE=65536     # 增加文件描述符限制
```

#### 启动脚本 (`iiri-ros/start_ros2_iiri_start.sh`)

```bash
# Enable coredump for debugging
ulimit -c unlimited
echo "Coredump enabled: $(ulimit -c)"
```

### 3. 编译配置

#### ARM 编译模式

```bash
# script/build_layered.sh
CMAKE_BUILD_TYPE="RelWithDebInfo"  # 优化 + 调试符号
```

**RelWithDebInfo** 特点：
- ✅ 保留调试符号（行号、变量名）
- ✅ 开启编译优化（-O2）
- ✅ 性能接近 Release
- ✅ 支持 coredump 分析

#### backward_ros 集成

修改的包：
- `motion_control` (motion_control_node, arm_ctrl_node)
- `robot_base` (robot_base_node)

**package.xml**:
```xml
<depend>backward_ros</depend>
```

**CMakeLists.txt**:
```cmake
find_package(backward_ros REQUIRED)
```

---

## 使用方法

### 查看崩溃列表

#### 使用 coredumpctl（推荐）

```bash
# 查看所有崩溃
coredumpctl list

# 查看今天的崩溃
coredumpctl list --since=today

# 查看最近1天的崩溃
coredumpctl list --since='1 day ago'

# 使用封装脚本
./tools/show_crashes.sh
./tools/show_crashes.sh --today
./tools/show_crashes.sh --since '1 day ago'
```

#### 直接查看文件

```bash
# 查看 /var/coredumps 目录
ls -lht /var/coredumps/

# 查看 systemd 管理的 coredump
ls -lh /var/lib/systemd/coredump/
```

### 分析 Coredump

#### 方法 1: 使用自动化脚本（最简单）

```bash
# 自动分析并生成报告
./tools/analyze_coredump.sh /var/coredumps/core-motion_control_node-12345-1729483200

# 脚本会自动：
# 1. 从 core 文件提取可执行文件路径
# 2. 使用 gdb 进行全面分析
# 3. 生成详细报告 (.analysis.txt)
# 4. 显示关键信息摘要
```

输出示例：
```
[INFO] 分析 core 文件: /var/coredumps/core-motion_control_node-12345-1729483200
[INFO] 检测到可执行文件: /home/wl/autorun/iiri-ros/install/lib/motion_control/motion_control_node
[INFO] 生成分析报告: /var/coredumps/core-motion_control_node-12345-1729483200.analysis.txt
[INFO] 开始 gdb 分析...
[INFO] 分析完成！

========================================
  关键信息摘要
========================================

[INFO] 崩溃信号:
Program terminated with signal SIGSEGV, Segmentation fault.

[INFO] 栈顶帧 (Top 5 frames):
#0  0x0000aaaae5c4b3c0 in iiri::qr::SDK::SetRunState()
#1  0x0000aaaae5c4a890 in qr_node::RxSetRunState()
#2  0x0000ffffb2e45678 in rclcpp::Subscription::handle_message()
#3  0x0000ffffb2e43210 in rclcpp::Executor::spin()
#4  0x0000aaaae5c49000 in main()
```

#### 方法 2: 使用 coredumpctl（直接调试）

```bash
# 查看崩溃详情
coredumpctl info 12345

# 直接用 gdb 调试
coredumpctl gdb 12345

# 提取 core 文件
coredumpctl dump 12345 -o core.12345

# 使用封装脚本
./tools/show_crashes.sh info 12345
./tools/show_crashes.sh dump 12345
```

#### 方法 3: 手动 gdb 分析（最灵活）

```bash
# 启动 gdb
gdb /path/to/executable /path/to/core

# 常用 gdb 命令
(gdb) bt full              # 完整回溯（包含局部变量）
(gdb) info threads         # 查看所有线程
(gdb) thread <N>           # 切换到线程 N
(gdb) frame <N>            # 切换到栈帧 N
(gdb) info locals          # 查看局部变量
(gdb) info args            # 查看函数参数
(gdb) p variable_name      # 打印变量值
(gdb) x/10x address        # 查看内存
(gdb) disassemble          # 反汇编当前函数
(gdb) info registers       # 查看寄存器
```

### 查看 backward_ros 堆栈

当节点崩溃时，backward_ros 会自动打印详细堆栈到 stderr：

```bash
# 查看节点的 stderr 日志
tail -100 /tmp/ros2_logs/2025-*/motion_control_node-*-stderr.log

# 或使用 journalctl
journalctl -u iiri-ros.service | tail -100
```

**backward_ros 输出示例**:
```
Stack trace (most recent call last):
#10   Object "/home/wl/autorun/iiri-ros/install/lib/motion_control/motion_control_node", at 0xaaaae5c49000, in main
#9    Object "/usr/lib/aarch64-linux-gnu/librclcpp.so", at 0xffffb2e43210, in rclcpp::Executor::spin()
        Source "/opt/ros/humble/include/rclcpp/executor.hpp", line 245, in rclcpp::Executor::spin()
#8    Object "/usr/lib/aarch64-linux-gnu/librclcpp.so", at 0xffffb2e45678, in rclcpp::Subscription::handle_message()
        Source "/opt/ros/humble/include/rclcpp/subscription.hpp", line 178
#7    Object "/home/wl/autorun/iiri-ros/install/lib/motion_control/motion_control_node", at 0xaaaae5c4a890, in qr_node::RxSetRunState()
        Source "/home/wl/twh/workspace/iiri_ros2_architecture/src/hardware_layer/src/motion_control/src/qr.cpp", line 142
#6    Object "/home/wl/autorun/iiri-sdk/lib/libiiri.so", at 0xffffb1c4b3c0, in iiri::qr::SDK::SetRunState()
        [Source not available]
Segmentation fault (address: 0x0)
```

优势：
- ✅ 自动触发，无需手动分析
- ✅ 包含源代码文件和行号
- ✅ 直接输出到日志，易于查看
- ✅ 无需 coredump 文件

---

## 分析技巧

### 识别崩溃类型

| 信号 | 含义 | 常见原因 |
|------|------|---------|
| **SIGSEGV** | 段错误 | 空指针解引用、访问无效内存、栈溢出 |
| **SIGABRT** | 中止信号 | assert 失败、std::abort() 调用 |
| **SIGBUS** | 总线错误 | 未对齐访问、硬件故障 |
| **SIGFPE** | 浮点异常 | 除以零、浮点溢出 |
| **SIGILL** | 非法指令 | 代码损坏、错误的函数指针 |

### 分析步骤

#### 1. 查看崩溃信号

```bash
# 从 coredumpctl 查看
coredumpctl info <PID> | grep Signal

# 从 gdb 查看
(gdb) info program
```

#### 2. 分析调用栈

```bash
# 查看完整调用栈
(gdb) bt full

# 切换栈帧
(gdb) frame 7    # 切换到 #7 帧

# 查看该帧的源代码
(gdb) list
```

重点关注：
- 最顶层的崩溃点（frame #0）
- 第一个自己代码的帧（通常是问题所在）
- 函数参数和局部变量的值

#### 3. 检查变量值

```bash
# 查看局部变量
(gdb) info locals

# 查看特定变量
(gdb) p msg->value
(gdb) p quadruped_

# 查看指针指向的内容
(gdb) p *quadruped_

# 查看数组内容
(gdb) p velocity_cmd[0]@3   # 打印 3 个元素
```

#### 4. 检查内存

```bash
# 查看内存内容（16 进制）
(gdb) x/10x 0xaaaae5c4b3c0

# 查看字符串
(gdb) x/s 0xaaaae5c4b3c0

# 查看指令
(gdb) x/10i $pc
```

#### 5. 多线程分析

```bash
# 查看所有线程
(gdb) info threads

# 切换线程
(gdb) thread 3

# 查看所有线程的栈
(gdb) thread apply all bt
```

### 常见问题定位

#### 空指针解引用

```cpp
// 问题代码
auto ret = quadruped_->SetRunState(tmp);  // quadruped_ 为 nullptr

// gdb 分析
(gdb) p quadruped_
$1 = (iiri::qr::SDK *) 0x0    <-- 空指针！
```

解决方法：在使用前检查指针：
```cpp
if (quadruped_ == nullptr) {
    RCLCPP_ERROR(this->get_logger(), "SDK not initialized!");
    return;
}
```

#### 访问已释放的内存

```cpp
// 问题代码
std::shared_ptr<MyClass> ptr;
{
    ptr = std::make_shared<MyClass>();
}
ptr->doSomething();  // ptr 已过期

// gdb 分析
(gdb) p ptr.use_count()
$1 = 0    <-- 引用计数为 0！
```

#### 数组越界

```cpp
// 问题代码
double velocity_cmd[3];
velocity_cmd[5] = 1.0;  // 越界！

// gdb 分析
(gdb) p &velocity_cmd
$1 = (double (*)[3]) 0xfffffffff000
(gdb) p &velocity_cmd[5]
$2 = (double *) 0xfffffffff028   <-- 超出数组范围！
```

---

## 常见问题

### Q1: 为什么没有生成 coredump 文件？

**检查步骤**:

1. **验证 ulimit 设置**:
   ```bash
   ulimit -c
   # 应该输出: unlimited
   ```

2. **检查 core_pattern**:
   ```bash
   cat /proc/sys/kernel/core_pattern
   # 应该输出: /var/coredumps/core-%e-%p-%t
   ```

3. **检查目录权限**:
   ```bash
   ls -ld /var/coredumps
   # 应该是: drwxrwxrwt (1777)
   ```

4. **查看 systemd 日志**:
   ```bash
   journalctl -u systemd-coredump.socket
   ```

### Q2: gdb 显示 "No debugging symbols found"

**原因**: 二进制文件没有调试符号

**解决方法**:

1. 确认使用 RelWithDebInfo 编译:
   ```bash
   grep CMAKE_BUILD_TYPE script/build_layered.sh
   # 应该看到: RelWithDebInfo
   ```

2. 重新编译:
   ```bash
   ./build_layered.sh -c arm hardware_layer
   ./deploy_package.sh arm
   ```

3. 验证符号:
   ```bash
   file /home/wl/autorun/iiri-ros/install/lib/motion_control/motion_control_node
   # 应该包含: "not stripped"
   ```

### Q3: backward_ros 堆栈没有源代码行号

**原因**: 缺少调试符号或源代码路径不匹配

**解决方法**:

1. 使用 RelWithDebInfo 编译（见 Q2）

2. 确保部署时保留源代码路径结构

3. 如果在不同机器上编译和运行，需要保持相同的路径

### Q4: coredump 文件太大，占满磁盘

**解决方法**:

1. 调整 systemd-coredump 限制:
   ```bash
   # 编辑 /etc/systemd/coredump.conf.d/custom.conf
   ProcessSizeMax=1G     # 降低单文件限制
   ExternalSizeMax=5G    # 降低总大小限制
   MaxUse=10G            # 降低磁盘使用上限
   ```

2. 手动清理旧文件:
   ```bash
   # 清理 7 天前的 coredump
   find /var/coredumps -name "core-*" -mtime +7 -delete
   find /var/lib/systemd/coredump -name "*.zst" -mtime +7 -delete
   ```

3. 配置自动清理（cron）:
   ```bash
   # 添加到 crontab
   0 2 * * * find /var/coredumps -name "core-*" -mtime +7 -delete
   ```

### Q5: 如何在远程服务器上调试？

**方法 1**: 使用分析脚本，生成报告后下载:
```bash
# 远程服务器
./tools/analyze_coredump.sh /var/coredumps/core-xxx
scp /var/coredumps/core-xxx.analysis.txt local:~/

# 本地查看
cat ~/core-xxx.analysis.txt
```

**方法 2**: SSH 转发 X11，远程运行 gdb GUI:
```bash
ssh -X wl@192.168.1.54
gdb-multiarch /path/to/executable /path/to/core
```

**方法 3**: 下载 core 和二进制文件到本地分析:
```bash
# 下载文件
scp wl@192.168.1.54:/var/coredumps/core-xxx ./
scp wl@192.168.1.54:/home/wl/autorun/iiri-ros/install/lib/motion_control/motion_control_node ./

# 本地分析（需要 gdb-multiarch）
gdb-multiarch motion_control_node core-xxx
```

---

## 最佳实践

### 开发阶段

1. **启用详细日志**
   ```cpp
   RCLCPP_DEBUG(get_logger(), "Variable x = %d", x);
   ```

2. **使用断言**
   ```cpp
   assert(quadruped_ != nullptr && "SDK must be initialized!");
   ```

3. **添加空指针检查**
   ```cpp
   if (!msg) {
       RCLCPP_ERROR(get_logger(), "Received null message!");
       return;
   }
   ```

4. **使用智能指针**
   ```cpp
   std::shared_ptr<SDK> quadruped_;  // 自动管理生命周期
   ```

### 测试阶段

1. **故意触发崩溃测试 coredump 功能**
   ```cpp
   // 测试代码
   int* p = nullptr;
   *p = 42;  // 触发 SIGSEGV
   ```

2. **验证 backward_ros 输出**
   ```bash
   tail -100 /tmp/ros2_logs/*/node-*-stderr.log
   ```

3. **验证 coredump 生成**
   ```bash
   ls -lh /var/coredumps/
   coredumpctl list
   ```

### 生产环境

1. **定期检查崩溃**
   ```bash
   # 每天检查
   ./tools/show_crashes.sh --since='1 day ago'
   ```

2. **设置磁盘监控**
   ```bash
   # 监控 /var/coredumps 使用率
   df -h /var/coredumps/
   ```

3. **保留重要 coredump**
   ```bash
   # 重命名关键 coredump
   mv core-xxx core-xxx-critical-issue-123.KEEP
   ```

4. **建立崩溃数据库**
   - 记录崩溃时间、版本、堆栈
   - 分析崩溃频率和模式
   - 优先修复高频崩溃

### 调试技巧

1. **对比崩溃前后的日志**
   ```bash
   journalctl -u iiri-ros.service --since='10 minutes ago' | grep -C 10 "SIGSEGV"
   ```

2. **查看系统资源**
   ```bash
   # 查看内存使用
   free -h
   # 查看进程内存
   ps aux | grep motion_control_node
   ```

3. **复现问题**
   - 记录崩溃时的输入和状态
   - 在开发环境中尝试复现
   - 使用 valgrind 检测内存问题

4. **使用版本控制定位引入问题的提交**
   ```bash
   git bisect start
   git bisect bad HEAD        # 当前版本有问题
   git bisect good v1.2.0     # v1.2.0 没问题
   # git 会自动二分查找问题提交
   ```

---

## 相关资源

### 工具文档

- [GDB 官方文档](https://sourceware.org/gdb/documentation/)
- [backward-cpp GitHub](https://github.com/bombela/backward-cpp)
- [systemd-coredump 文档](https://www.freedesktop.org/software/systemd/man/systemd-coredump.html)

### 内部文档

- [CLAUDE.md](../../CLAUDE.md) - 项目开发指南
- [build-system.md](../development/build-system.md) - 编译系统说明
- [troubleshooting.md](../reference/troubleshooting.md) - 常见问题排查

### 支持

如有问题，请联系：
- **作者**: 唐文浩
- **Email**: twh@example.com
- **Issue**: 在 Git 仓库提交 Issue

---

**最后更新**: 2025-10-21
**文档版本**: v1.0.0

# Path Tracker 架构说明

> **文档位置**: `docs/architecture/path-tracker.md`
> **最后更新**: 2025-10-13
> **作者**: 唐文浩

## 概述

`path_tracker` 包现在支持两种编译模式：
1. **基础模式**（默认）：使用简单几何计算，不依赖 Ceres 优化库
2. **优化模式**：使用 Ceres 非线性优化库进行路径跟踪优化

## 架构设计

### 1. 条件编译机制

通过 CMake 选项 `USE_CERES_OPTIMIZATION` 控制是否启用 Ceres 优化：
- `OFF`（默认）：禁用 Ceres，使用基础几何计算
- `ON`：启用 Ceres 优化，与原 wl_ros 项目保持一致

### 2. 代码结构

#### SimplePathTracker 类
- **基础模式**：使用 `std::atan2` 进行简单角速度计算
- **优化模式**：使用 Ceres 求解器进行非线性优化

#### 条件编译标识
```cpp
#ifdef USE_CERES_OPTIMIZATION
    // Ceres 优化代码
#else
    // 基础几何计算代码
#endif
```

## 编译方式

### 方法一：使用主编译脚本（推荐）

#### 禁用 Ceres 优化（默认）
```bash
# 编译所有层
./build_layered.sh

# 编译特定层（如 Application Layer）
./build_layered.sh application_layer

# 清理并重新编译
./build_layered.sh -c application_layer
```

#### 启用 Ceres 优化
```bash
# 编译所有层并启用 Ceres
./build_layered.sh --ceres

# 编译特定层并启用 Ceres
./build_layered.sh --ceres application_layer

# 清理并重新编译，启用 Ceres
./build_layered.sh -c --ceres application_layer
```

### 方法二：使用便捷脚本

#### 启用 Ceres 的便捷脚本
```bash
# 自动启用 Ceres 编译所有层
./build_layered_ceres.sh

# 自动启用 Ceres 编译特定层
./build_layered_ceres.sh application_layer

# 清理并重新编译（启用 Ceres）
./build_layered_ceres.sh -c application_layer
```

### 方法三：传统分层编译（已弃用）
```bash
# 仅用于向后兼容，不推荐使用
./build_layered_with_ceres.sh intelligence_layer
```

**注意**：优化模式需要系统安装 Ceres 库，在没有 NVIDIA 驱动的环境下可能编译失败。

## 功能对比

| 特性 | 基础模式 | 优化模式 |
|------|----------|----------|
| 依赖库 | 仅 Eigen3 | Eigen3 + Ceres |
| 计算方法 | 几何计算 | 非线性优化 |
| 性能 | 快速 | 更精确但计算量大 |
| 兼容性 | 高（无外部依赖） | 需要 Ceres 库 |
| 与原项目一致性 | 简化版本 | 完全一致 |

## 参数配置

两种模式共享相同的参数配置：
- `track_forward_distance`: 前瞻距离
- `fixed_velocity`: 固定速度
- `weight_position`: 位置权重（仅优化模式使用）
- `weight_theta`: 角度权重（仅优化模式使用）

## 使用建议

1. **开发环境**：推荐使用基础模式，编译快速，依赖简单
2. **生产环境**：如果需要更高精度的路径跟踪，可使用优化模式
3. **资源受限环境**：使用基础模式，计算开销更小

## 故障排除

### Ceres 编译失败
如果启用 Ceres 模式编译失败，请检查：
1. 系统是否安装了 Ceres 库
2. 是否有 NVIDIA 驱动（某些 Ceres 功能需要）
3. 回退到基础模式：使用 `./build_layered.sh`

### 运行时问题
- 基础模式和优化模式的接口完全兼容
- 可以在运行时通过参数调整行为
- 两种模式的输出格式相同

## 架构优势

1. **向后兼容**：保持与原 wl_ros 项目的接口一致性
2. **灵活部署**：根据环境选择合适的编译模式
3. **分层清晰**：遵循项目的分层架构原则
4. **维护性好**：条件编译确保代码清晰分离
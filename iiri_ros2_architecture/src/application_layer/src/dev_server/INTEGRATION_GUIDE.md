# Robolink Frontend Integration Guide

## 概述

本文档说明如何使用已集成到 dev_server 中的 robolink 前端控制界面。

## 集成架构

### 前端 (Vue 3)
- **位置**: `frontend_src/`
- **编译输出**: `static/` (通过 `build_frontend.sh` 脚本生成)
- **框架**: Vue 3 + TypeScript + Element Plus + Pinia
- **WebSocket 端口**: 20555
- **HTTP API 端口**: 8080

### 后端 (C++ ROS2)
- **位置**: `src/`
- **主要文件**:
  - `dev_server_node.cpp` - 主节点，HTTP/WebSocket 服务器
  - `http_api.cpp/hpp` - HTTP API 处理器，提供设备 UUID 接口
  - `ws_server.cpp/hpp` - WebSocket 服务器

## 构建流程

### 1. 前端构建

```bash
cd src/application_layer/src/dev_server/frontend_src
npm install                # 首次或依赖变更时
npm run build             # 编译生产版本到 dist/
cp -r dist/* ../static/   # 复制到静态文件目录
```

或使用快捷脚本:

```bash
cd src/application_layer/src/dev_server/frontend_src
./build_frontend.sh
```

### 2. 后端构建

```bash
cd /path/to/iiri_ros2_architecture
./build_layered.sh x86 application_layer    # x86 架构
# 或
./build_layered.sh arm application_layer    # ARM 架构 (树莓派)
```

### 3. 完整部署（一键脚本）

```bash
cd /path/to/iiri_ros2_architecture
./deploy_dev_server_frontend.sh x86         # 本地 PC
./deploy_dev_server_frontend.sh arm deploy  # 编译并部署到树莓派
```

## 使用方法

### 启动服务

在机器狗/机器人上运行:

```bash
ros2 launch iiri_bringup iiri_system.launch.py
```

dev_server 会自动启动，监听:
- HTTP: `0.0.0.0:8080`
- WebSocket: `0.0.0.0:20555`

### 访问界面

#### PC 端
在浏览器中打开:
```
http://<机器人IP>:8080
```

#### 手机端
同样访问上述地址，前端已适配移动设备。

### 首次使用

1. **自动获取 UUID**: 页面加载时会自动从 `/api/device/uuid` 获取设备 UUID
2. **显示 UUID**: UUID 显示在连接面板的只读输入框中
3. **刷新 UUID**: 点击"刷新"按钮可重新获取
4. **建立连接**: 输入正确的 IP 后点击"连接"按钮

## API 接口

### HTTP REST API

#### 获取设备 UUID
```
GET /api/device/uuid
Response: {
  "uuid": "6f06e9cb0071",
  "deviceName": "IIRI Robot",
  "deviceType": "quadruped",
  "timestamp": 1697567890000
}
```

#### 获取设备完整信息
```
GET /api/device/info
Response: 同上
```

### WebSocket 频道

格式: `ws://<ip>:20555/<uuid>/<channel>`

可用频道:
- `/robot/video` - 视频流
- `/robot/audio` - 音频输出
- `/user/audio` - 音频输入
- `/device/set_camera_ptz` - 云台控制
- `/quadruped/set_velocity` - 速度控制
- `/quadruped/set_run_state` - 运行状态
- `/device/set_talk` - 对讲

## 文件结构

```
dev_server/
├── frontend_src/              # Vue 前端源代码
│   ├── src/
│   │   ├── api/
│   │   │   └── robot.ts       # API 调用层
│   │   ├── stores/
│   │   │   └── websocket.ts   # WebSocket 状态管理
│   │   ├── components/
│   │   │   └── ConnectionPanel.vue  # 连接面板组件
│   │   └── ...
│   ├── package.json
│   ├── vite.config.js
│   └── build_frontend.sh      # 前端构建脚本
│
├── static/                    # 前端编译输出（nginx 根目录）
│   ├── index.html
│   ├── assets/
│   └── ...
│
├── src/
│   ├── dev_server_node.cpp    # 主节点
│   ├── http_api.cpp/hpp       # HTTP API 处理
│   └── ws_server.cpp/hpp      # WebSocket 服务
│
├── CMakeLists.txt
└── INTEGRATION_GUIDE.md       # 本文档
```

## 核心修改说明

### 前端修改

1. **API 层** (`src/api/robot.ts`)
   - 添加 `fetchDeviceUUID()` 函数
   - 从 `/api/device/uuid` 获取设备 UUID

2. **WebSocket 配置** (`stores/websocket.ts`)
   - 修改 WebSocket URL 格式: `ws://{ip}:20555/{uuid}/{channel}`
   - 端口从 8080 改为 20555
   - 添加 `initUUID()` 方法自动获取 UUID
   - IP 自动使用 `window.location.hostname`

3. **连接面板** (`components/ConnectionPanel.vue`)
   - UUID 输入框改为只读
   - 添加"刷新"按钮手动重新获取 UUID
   - 页面加载时自动调用 `initUUID()`

### 后端修改

1. **HTTP API 处理器** (`http_api.cpp/hpp`)
   - 实现 `getDeviceUUIDJson()` 返回 JSON 格式的设备信息
   - 实现 `fetchUUIDFromRobotBase()` 调用 `/robot_base/my_info` ROS2 服务
   - 实现 `getSystemUUID()` 从 `/etc/machine-id` 读取备用 UUID

2. **主节点集成** (`dev_server_node.cpp`)
   - 引入 `http_api.hpp`
   - 实例化 `HttpApiHandler`
   - 注册 `/api/device/uuid` 和 `/api/device/info` 路由
   - 设置静态文件服务: `http_srv_.set_mount_point("/", "static")`

3. **构建配置** (`CMakeLists.txt`)
   - 添加 `src/http_api.cpp` 到 `NODE_SRC`

## UUID 获取机制

1. **优先级 1**: 调用 ROS2 服务 `/robot_base/my_info`
2. **优先级 2**: 读取 `/etc/machine-id` (前12位)
3. **优先级 3**: 返回 "unknown_device"

## 常见问题

### Q: 前端构建失败
A:
```bash
cd frontend_src
rm -rf node_modules package-lock.json
npm install
npm run build
```

### Q: UUID 无法获取
A: 检查 robot_base 节点是否运行:
```bash
ros2 service list | grep my_info
```

### Q: WebSocket 连接失败
A:
1. 检查防火墙是否开放 20555 端口
2. 确认 dev_server_node 正在运行
3. 验证 UUID 是否正确

### Q: 静态文件 404
A: 确保 static/ 目录存在且包含编译产物:
```bash
ls -la src/application_layer/src/dev_server/static/
```

## 开发提示

### 前端开发

开发模式（支持热重载）:
```bash
cd frontend_src
npm run dev
```
访问 `http://localhost:5173`

### 后端开发

修改 C++ 代码后重新编译:
```bash
./build_layered.sh x86 application_layer
```

### 部署测试

```bash
# 1. 编译前端
cd src/application_layer/src/dev_server/frontend_src
./build_frontend.sh

# 2. 编译后端
cd /path/to/iiri_ros2_architecture
./build_layered.sh arm application_layer

# 3. 部署到树莓派
sshpass -p '123456' scp -r iiri-ros/install/* wl@192.168.1.54:/opt/iiri-ros/

# 4. 重启服务
sshpass -p '123456' ssh wl@192.168.1.54 "sudo systemctl restart iiri-ros.service"
```

## 技术栈

### 前端
- Vue 3.5
- TypeScript 5.6
- Vite 6.4
- Element Plus 2.9
- Pinia (状态管理)

### 后端
- ROS2 Humble
- C++17
- cpp-httplib (HTTP 服务器)
- 自定义 WebSocket 实现

## 版本历史

- **v1.0.0** (2025-10-17): 初始集成
  - 移除 robolink 后端
  - 集成 Vue 前端到 dev_server
  - 实现自动 UUID 获取
  - 支持 PC 和移动端

## 相关文档

- [完整技术文档](docs/development/robolink-frontend-integration.md)
- [ROS2 架构文档](../../../README.md)
- [dev_server API 文档](./README.md)

## 联系方式

如有问题，请联系开发团队或查看项目 Issue。

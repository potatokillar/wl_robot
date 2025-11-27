# Vue 前端 OTA 更新实现总结

## 实现概述

已完成 Vue 前端的智能 OTA 更新功能，支持 **WebSocket 实时推送** 和 **HTTP 轮询** 的自动切换，确保在 ROS2 服务重启期间也能持续获取更新状态。

## 核心功能

### ✅ 1. 智能连接模式切换

```
更新流程：
┌─────────────────────────────────────────────────┐
│ 阶段1：WebSocket 实时模式                       │
│ ✓ 前端 ←→ WebSocket ←→ dev_server               │
│ ✓ 实时接收更新进度推送                          │
├─────────────────────────────────────────────────┤
│ 阶段2：ROS2 停止（WebSocket 断开）              │
│ ✗ WebSocket 断开                                │
│ → 自动切换到 HTTP 轮询模式                      │
│ → 显示"ROS2 服务正在更新中"                     │
├─────────────────────────────────────────────────┤
│ 阶段3：HTTP 轮询模式                            │
│ ⟳ 每 5 秒轮询 /api/update/current-status       │
│ ✓ 继续获取更新进度                             │
│ ✓ 显示轮询进度条                               │
├─────────────────────────────────────────────────┤
│ 阶段4：ROS2 重启（恢复连接）                    │
│ ✓ 检测到 dev_server 重启                       │
│ ✓ 自动重连 WebSocket                           │
│ ✓ 停止 HTTP 轮询                               │
│ ✓ 显示最终结果                                 │
└─────────────────────────────────────────────────┘
```

### ✅ 2. 包名验证提示

在 UI 中添加了友好的提示信息：
- **ROS2 包**：文件名必须包含 "ros" 或 "ROS"
- **QR 包**：文件名必须包含 "qr" 或 "QR"

上传时如果不符合规则，后端会返回详细错误信息。

### ✅ 3. 连接状态可视化

**连接状态标签**：
- 🟢 **WebSocket 已连接** - 正常实时模式
- 🟡 **HTTP 轮询模式** - ROS2 更新中
- ⚪ **WebSocket 断开** - 连接中断

**状态提示框**：
- ⚠️ **ROS2 服务正在更新中** - 显示轮询进度
- 📊 **轮询进度条** - 显示重连尝试次数

### ✅ 4. 完整的进度追踪

**实时显示**：
- 📊 进度百分比
- 🏷️ 当前状态（准备中/更新中/成功/失败）
- 💬 当前阶段消息
- ⏱️ 用时统计

**状态颜色**：
- 🟢 SUCCESS / ROLLBACK_SUCCESS - 绿色
- 🔴 FAILED / ROLLBACK_FAILED - 红色
- 🟡 RUNNING / PREPARING - 黄色
- ⚪ 其他状态 - 灰色

## 修改的文件

### 1. `src/api/update.ts`
添加了 `getCurrentStatus()` 方法：

```typescript
/**
 * 获取当前更新状态（直接读取状态文件，用于 WebSocket 断开后的轮询）
 */
export const getCurrentStatus = async (): Promise<StatusResponse> => {
  const response = await axios.get(`${API_BASE_URL}/api/update/current-status`, {
    timeout: 5000
  })
  return response.data
}
```

### 2. `src/views/OTAUpdate.vue`
完全重写，添加了以下核心功能：

#### 核心状态管理
```typescript
// 连接模式
const isWebSocketMode = ref(true)  // true: WebSocket, false: HTTP 轮询

// HTTP 轮询
const pollingInterval = ref<number | null>(null)
const pollingActive = ref(false)
const reconnectAttempts = ref(0)
const maxReconnectAttempts = ref(60)  // 最多 5 分钟
```

#### 关键函数

**1. startPolling() - 启动 HTTP 轮询**
```typescript
const startPolling = () => {
  pollingInterval.value = setInterval(async () => {
    reconnectAttempts.value++
    
    // 获取当前状态
    const response = await getCurrentStatus()
    
    if (response.success) {
      updateTaskStatus(response.status)
      
      // 更新完成，尝试重连 WebSocket
      if (isUpdateComplete(response.status.state)) {
        await attemptReconnectWebSocket()
      }
    }
    
    // 超时处理
    if (reconnectAttempts.value >= maxReconnectAttempts.value) {
      stopPolling()
      ElMessage.error('更新状态查询超时')
    }
  }, 5000)
}
```

**2. watch() - 监听 WebSocket 状态变化**
```typescript
watch(() => wsStore.isConnected, (connected, wasConnectedBefore) => {
  // WebSocket 断开 + 正在更新 → 启动 HTTP 轮询
  if (!connected && wasConnectedBefore && updating.value) {
    ElMessage.warning('连接中断，已切换到轮询模式')
    startPolling()
  }
  
  // WebSocket 重连 → 停止 HTTP 轮询
  if (connected && !wasConnectedBefore && pollingActive.value) {
    ElMessage.success('连接已恢复')
    stopPolling()
  }
})
```

**3. attemptReconnectWebSocket() - 重连 WebSocket**
```typescript
const attemptReconnectWebSocket = async () => {
  if (wsStore.isConnected) {
    stopPolling()
    return true
  }
  
  await wsStore.connect()
  
  if (wsStore.isConnected) {
    ElMessage.success('连接已恢复')
    stopPolling()
    return true
  }
  
  return false
}
```

## UI 增强

### 1. 连接状态区域
```vue
<div class="header-status">
  <el-tag :type="connectionStatusColor" size="small">
    {{ connectionStatusText }}
  </el-tag>
  <el-tag :type="statusColor">{{ statusText }}</el-tag>
</div>
```

### 2. 连接提示框
```vue
<el-alert
  v-if="showConnectionAlert"
  :title="connectionAlertTitle"
  :type="connectionAlertType"
>
  <div>{{ connectionAlertMessage }}</div>
  <el-progress
    v-if="pollingActive"
    :percentage="pollingProgress"
    :format="() => `轮询中 (${reconnectAttempts}/${maxReconnectAttempts})`"
    :indeterminate="true"
  />
</el-alert>
```

### 3. 包名验证提示
```vue
<div class="app-type-hint">
  <el-text size="small" type="info">
    ROS2 包文件名必须包含 "ros" 或 "ROS"，QR 包必须包含 "qr" 或 "QR"
  </el-text>
</div>
```

## 工作流程详解

### 场景 1：正常更新流程（WebSocket 始终连接）

```
1. 用户上传文件 → uploadFiles()
   ✓ 上传到 /api/update/upload
   ✓ 获取 taskId

2. 用户点击"开始更新" → startUpdate()
   ✓ 调用 /api/update/start
   ✓ updating.value = true

3. WebSocket 接收实时进度
   ✓ handleWebSocketMessage() 处理
   ✓ updateTaskStatus() 更新 UI
   ✓ 进度条实时更新

4. 更新完成
   ✓ updating.value = false
   ✓ 显示成功/失败消息
```

### 场景 2：ROS2 重启期间的更新（WebSocket 断开）

```
1. 用户开始更新
   ✓ WebSocket 正常接收进度

2. update_manager 执行 systemctl stop iiri-ros
   ✗ dev_server 停止
   ✗ WebSocket 断开
   
3. watch() 检测到 WebSocket 断开
   ✓ 触发 startPolling()
   ✓ 显示"ROS2 服务正在更新中"
   ✓ 开始 HTTP 轮询（每 5 秒）

4. HTTP 轮询持续获取状态
   ⟳ 调用 getCurrentStatus()
   ✓ 读取 /var/run/update_status.json
   ✓ 更新 UI 进度

5. ROS2 重启完成
   ✓ dev_server 启动
   ✓ 轮询检测到服务恢复
   ✓ attemptReconnectWebSocket()
   ✓ 恢复 WebSocket 连接
   ✓ 停止 HTTP 轮询
   ✓ 显示最终结果
```

### 场景 3：轮询超时处理

```
如果轮询超过 60 次（5 分钟）仍无法连接：
✓ 停止轮询
✓ 显示超时错误
✓ 提示用户手动刷新或检查设备
```

## 容错机制

### 1. 文件上传验证
- ✅ 前端检查：必须选择 .tar.gz 和 .sha256 文件
- ✅ 后端验证：文件名必须符合 appType 规则
- ✅ 错误提示：显示详细的验证失败原因

### 2. 连接失败处理
- ✅ WebSocket 断开：自动切换到 HTTP 轮询
- ✅ HTTP 轮询失败：继续重试，直到超时
- ✅ 超时保护：最多尝试 5 分钟

### 3. 状态同步
- ✅ 任务 ID 验证：确保状态属于当前任务
- ✅ 状态去重：防止重复更新 UI
- ✅ 完成检测：自动停止轮询

### 4. 用户反馈
- ✅ 实时进度显示
- ✅ 连接状态可视化
- ✅ 详细错误消息
- ✅ 操作确认提示

## 测试建议

### 1. 正常流程测试
```bash
# 上传正确命名的文件
✓ iiri-ros-arm-v1.0.0.tar.gz + appType=ros2  # 应该成功
✓ iiri-qr-v1.0.0.tar.gz + appType=qr         # 应该成功
```

### 2. 错误处理测试
```bash
# 上传错误命名的文件
✗ iiri-qr-v1.0.0.tar.gz + appType=ros2  # 应该被拒绝
✗ iiri-ros-v1.0.0.tar.gz + appType=qr   # 应该被拒绝
✗ update-v1.0.0.tar.gz + appType=ros2   # 应该被拒绝
```

### 3. 连接切换测试
```bash
# 1. 开始更新，观察 WebSocket 实时推送
# 2. ROS2 停止，WebSocket 断开
#    ✓ 应该自动切换到"HTTP 轮询模式"
#    ✓ 显示轮询进度条
# 3. ROS2 重启，dev_server 恢复
#    ✓ 应该自动恢复 WebSocket 连接
#    ✓ 显示最终结果
```

### 4. 超时测试
```bash
# 模拟长时间无响应
# ✓ 应该在 5 分钟后显示超时错误
# ✓ 停止轮询
```

## 配置说明

可在组件中调整以下参数：

```typescript
// 轮询间隔（毫秒）
const POLLING_INTERVAL = 5000  // 默认 5 秒

// 最大重试次数
const maxReconnectAttempts = 60  // 默认 60 次 = 5 分钟

// HTTP 超时
const HTTP_TIMEOUT = 5000  // 默认 5 秒
```

## 依赖关系

### 前端依赖
- Vue 3
- Element Plus
- Pinia（状态管理）
- Axios（HTTP 请求）

### 后端 API
- `POST /api/update/upload` - 上传文件
- `POST /api/update/start` - 启动更新
- `GET /api/update/status/{taskId}` - 查询指定任务状态
- `GET /api/update/current-status` - 查询当前活动状态（新增）
- `POST /api/update/cancel/{taskId}` - 取消更新
- `WebSocket ws://{ip}:20555/{uuid}/usr/robot/video` - 实时推送

## 已知限制

1. **轮询期间无实时进度**：ROS2 停止期间，前端只能通过轮询获取状态（5 秒间隔）
   - 影响：可能会有最多 5 秒的延迟
   - 缓解：已添加进度条和提示信息

2. **超时时间固定**：当前设置为 5 分钟（60 次 × 5 秒）
   - 如果更新时间超过 5 分钟，需要手动增加 `maxReconnectAttempts`

3. **WebSocket 端口固定**：目前硬编码为 20555
   - 如果需要修改，需要同时修改 `websocket.ts` 中的配置

## 后续改进建议

### 优先级 1（可选）
- [ ] 添加"查看日志"功能，直接显示 `/tmp/updates/{taskId}.log`
- [ ] 支持同时监控多个更新任务
- [ ] 添加更新历史记录

### 优先级 2（高级）
- [ ] 实现 Nginx 轮询方案（方案 B），获取更实时的进度
- [ ] 添加更新包预验证（上传前检查文件完整性）
- [ ] 支持增量更新

### 优先级 3（性能）
- [ ] 优化轮询频率（根据状态动态调整）
- [ ] 添加连接质量监测
- [ ] 实现断点续传

## 总结

✅ **已完成**：
- WebSocket 实时推送
- HTTP 轮询自动切换
- 智能重连机制
- 完整的 UI 反馈
- 包名验证和错误处理

✅ **测试建议**：
- 完整更新流程测试
- 错误场景测试
- 连接断开恢复测试
- 超时处理测试

✅ **用户体验**：
- 实时进度显示
- 清晰的状态指示
- 友好的错误提示
- 无需手动干预的自动切换

现在可以进行完整的测试和部署了！🎉

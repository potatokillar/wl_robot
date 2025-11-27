<template>
  <div class="ota-update">
    <el-card>
      <template #header>
        <div class="card-header">
          <span>OTA 系统更新</span>
          <div class="header-status">
            <el-tag :type="connectionStatusColor" size="small">
              {{ connectionStatusText }}
            </el-tag>
            <el-tag :type="statusColor" style="margin-left: 10px;">{{ statusText }}</el-tag>
          </div>
        </div>
      </template>

      <!-- 文件上传区 -->
      <el-upload
        ref="uploadRef"
        :auto-upload="false"
        :on-change="handleFileChange"
        :limit="2"
        accept=".tar.gz,.sha256"
        drag
        multiple
      >
        <el-icon class="el-icon--upload"><UploadFilled /></el-icon>
        <div class="el-upload__text">
          拖拽文件到此处或<em>点击上传</em>
        </div>
        <template #tip>
          <div class="el-upload__tip">
            请上传 .tar.gz 更新包和对应的 .sha256 校验文件
          </div>
        </template>
      </el-upload>

      <!-- 上传进度条 -->
      <div v-if="uploading" class="upload-progress">
        <el-progress
          :percentage="uploadProgress"
          :stroke-width="20"
          :status="uploadProgress === 100 ? 'success' : undefined"
        >
          <template #default="{ percentage }">
            <span class="progress-text">{{ percentage }}%</span>
          </template>
        </el-progress>
        <div class="upload-tip">
          <el-text size="small" type="info">
            正在上传文件，请稍候...（{{ formatFileSize(uploadedSize) }} / {{ formatFileSize(totalSize) }}）
          </el-text>
        </div>
      </div>

      <!-- 应用类型选择 -->
      <el-form :model="form" style="margin-top: 20px;">
        <el-form-item label="应用类型">
          <el-radio-group v-model="form.appType" :disabled="!!currentTask">
            <el-radio value="ros2">ROS2 集群</el-radio>
            <el-radio value="qr">QR 控制进程</el-radio>
          </el-radio-group>
          <div class="app-type-hint">
            <el-text size="small" type="info">
              ROS2 包文件名必须包含 "ros" 或 "ROS"，QR 包必须包含 "qr" 或 "QR"
            </el-text>
          </div>
        </el-form-item>

        <el-form-item label="后台运行">
          <el-switch v-model="form.daemon" />
        </el-form-item>
      </el-form>

      <!-- 操作按钮 -->
      <div class="button-group">
        <el-button
          type="primary"
          :loading="uploading"
          :disabled="!canUpload"
          @click="uploadFiles"
        >
          上传文件
        </el-button>

        <el-button
          type="success"
          :loading="updating"
          :disabled="!canStartUpdate"
          @click="startUpdate"
        >
          开始更新
        </el-button>

        <el-button
          type="danger"
          :disabled="!canCancel"
          @click="cancelUpdate"
        >
          取消更新
        </el-button>
      </div>

      <!-- 连接状态提示 -->
      <el-alert
        v-if="showConnectionAlert"
        :title="connectionAlertTitle"
        :type="connectionAlertType"
        :closable="false"
        style="margin-top: 20px;"
      >
        <template #default>
          <div>{{ connectionAlertMessage }}</div>
          <el-progress
            v-if="pollingActive"
            :percentage="pollingProgress"
            :format="() => `轮询中 (${reconnectAttempts}/${maxReconnectAttempts})`"
            :indeterminate="true"
            style="margin-top: 10px;"
          />
        </template>
      </el-alert>

      <!-- 进度显示 -->
      <div v-if="currentTask" class="progress-section">
        <el-divider />

        <h3>更新进度</h3>

        <el-progress
          :percentage="progress"
          :status="progressStatus"
          :stroke-width="20"
        />

        <div class="status-info">
          <el-descriptions :column="2" border>
            <el-descriptions-item label="任务 ID">
              {{ currentTask.taskId }}
            </el-descriptions-item>
            <el-descriptions-item label="应用类型">
              <el-tag :type="currentTask.appType === 'ros2' ? 'primary' : 'success'">
                {{ currentTask.appType === 'ros2' ? 'ROS2 集群' : 'QR 控制进程' }}
              </el-tag>
            </el-descriptions-item>
            <el-descriptions-item label="状态">
              <el-tag :type="statusColor">{{ statusText }}</el-tag>
            </el-descriptions-item>
            <el-descriptions-item label="当前阶段">
              {{ currentTask.message }}
            </el-descriptions-item>
            <el-descriptions-item label="用时" :span="2">
              {{ currentTask.duration || 0 }} 秒
            </el-descriptions-item>
          </el-descriptions>
        </div>

        <!-- 错误信息 -->
        <el-alert
          v-if="currentTask.errorMsg"
          :title="currentTask.errorMsg"
          type="error"
          show-icon
          :closable="false"
          style="margin-top: 20px;"
        />
      </div>
    </el-card>
  </div>
</template>

<script setup lang="ts">
import { ref, computed, onMounted, onUnmounted, watch } from 'vue'
import { ElMessage } from 'element-plus'
import { UploadFilled } from '@element-plus/icons-vue'
import { useWebSocketStore } from '@/stores/websocket'
import { 
  uploadUpdatePackage, 
  startUpdateTask, 
  cancelUpdateTask,
  getCurrentStatus,
  type UpdateStatus 
} from '@/api/update'

// WebSocket 连接
const wsStore = useWebSocketStore()

// 表单数据
const form = ref({
  appType: 'ros2',
  daemon: true
})

// 上传的文件
const uploadRef = ref()
const packageFile = ref<File | null>(null)
const sha256File = ref<File | null>(null)

// 任务状态
const currentTask = ref<UpdateStatus | null>(null)
const uploading = ref(false)
const updating = ref(false)

// 上传进度
const uploadProgress = ref(0)
const uploadedSize = ref(0)
const totalSize = ref(0)

// WebSocket 连接状态
const wasConnected = ref(false)
const isWebSocketMode = ref(true) // true: WebSocket 模式, false: HTTP 轮询模式

// HTTP 轮询相关
const pollingInterval = ref<number | null>(null)
const pollingActive = ref(false)
const reconnectAttempts = ref(0)
const maxReconnectAttempts = ref(60) // 最多尝试 5 分钟（每 5 秒一次）

// 计算属性
const canUpload = computed(() => {
  return packageFile.value && sha256File.value && !uploading.value
})

const canStartUpdate = computed(() => {
  return currentTask.value?.taskId && !updating.value &&
         currentTask.value?.state !== 'RUNNING' &&
         currentTask.value?.state !== 'PREPARING'
})

const canCancel = computed(() => {
  return currentTask.value?.state === 'RUNNING' ||
         currentTask.value?.state === 'PREPARING'
})

const progress = computed(() => {
  return currentTask.value?.progress || 0
})

const statusColor = computed(() => {
  const state = currentTask.value?.state
  switch (state) {
    case 'SUCCESS':
    case 'ROLLBACK_SUCCESS':
      return 'success'
    case 'FAILED':
    case 'ROLLBACK_FAILED':
      return 'danger'
    case 'RUNNING':
    case 'PREPARING':
      return 'warning'
    default:
      return 'info'
  }
})

const progressStatus = computed(() => {
  const state = currentTask.value?.state
  if (state === 'SUCCESS' || state === 'ROLLBACK_SUCCESS') return 'success'
  if (state === 'FAILED' || state === 'ROLLBACK_FAILED') return 'exception'
  return undefined
})

const statusText = computed(() => {
  const state = currentTask.value?.state
  const stateMap: Record<string, string> = {
    'CREATED': '已创建',
    'PREPARING': '准备中',
    'RUNNING': '更新中',
    'SUCCESS': '成功',
    'FAILED': '失败',
    'ROLLBACK': '回滚中',
    'ROLLBACK_SUCCESS': '回滚成功',
    'ROLLBACK_FAILED': '回滚失败',
    'CANCELLED': '已取消'
  }
  return stateMap[state || ''] || '未知'
})

// 连接状态
const connectionStatusColor = computed(() => {
  if (wsStore.isConnected && isWebSocketMode.value) return 'success'
  if (pollingActive.value) return 'warning'
  return 'info'
})

const connectionStatusText = computed(() => {
  if (wsStore.isConnected && isWebSocketMode.value) return 'WebSocket 已连接'
  if (pollingActive.value) return 'HTTP 轮询模式'
  return 'WebSocket 断开'
})

const showConnectionAlert = computed(() => {
  return !wsStore.isConnected || pollingActive.value
})

const connectionAlertTitle = computed(() => {
  if (pollingActive.value) {
    return 'ROS2 服务正在更新中'
  }
  return 'WebSocket 连接已断开'
})

const connectionAlertType = computed(() => {
  if (pollingActive.value) return 'warning'
  return 'info'
})

const connectionAlertMessage = computed(() => {
  if (pollingActive.value) {
    return '设备正在重启，已切换到 HTTP 轮询模式获取更新状态。请耐心等待，这可能需要几分钟...'
  }
  return '连接中断，正在尝试重新连接...'
})

const pollingProgress = computed(() => {
  return Math.min((reconnectAttempts.value / maxReconnectAttempts.value) * 100, 100)
})

// 文件选择处理
const handleFileChange = (file: any) => {
  const filename = file.name.toLowerCase()
  if (filename.endsWith('.tar.gz')) {
    packageFile.value = file.raw
  } else if (filename.endsWith('.sha256')) {
    sha256File.value = file.raw
  }
}

// 格式化文件大小
const formatFileSize = (bytes: number): string => {
  if (bytes === 0) return '0 B'
  const k = 1024
  const sizes = ['B', 'KB', 'MB', 'GB']
  const i = Math.floor(Math.log(bytes) / Math.log(k))
  return Math.round(bytes / Math.pow(k, i) * 100) / 100 + ' ' + sizes[i]
}

// 上传文件
const uploadFiles = async () => {
  if (!packageFile.value || !sha256File.value) {
    ElMessage.error('请选择更新包和校验文件')
    return
  }

  // 重置上传进度
  uploadProgress.value = 0
  uploadedSize.value = 0
  totalSize.value = packageFile.value.size + sha256File.value.size

  uploading.value = true
  try {
    const formData = new FormData()
    formData.append('file', packageFile.value, packageFile.value.name)
    formData.append('sha256_file', sha256File.value, sha256File.value.name)
    formData.append('app_type', form.value.appType)

    console.log('上传参数:', {
      fileSize: packageFile.value.size,
      fileName: packageFile.value.name,
      sha256Size: sha256File.value.size,
      sha256Name: sha256File.value.name,
      appType: form.value.appType,
      totalSize: totalSize.value
    })

    const response = await uploadUpdatePackage(formData, (progress) => {
      uploadProgress.value = progress
      uploadedSize.value = Math.round(totalSize.value * progress / 100)
    })

    if (response.success) {
      uploadProgress.value = 100
      uploadedSize.value = totalSize.value

      currentTask.value = {
        taskId: response.taskId,
        appType: form.value.appType,  // 保存上传时的 appType
        state: 'CREATED',
        progress: 0,
        message: '文件上传完成，等待开始更新'
      }
      ElMessage.success('文件上传成功')
    } else {
      // 上传失败时清空任务状态，防止使用旧的 taskId
      currentTask.value = null
      packageFile.value = null
      sha256File.value = null
      uploadProgress.value = 0
      ElMessage.error(response.message || '上传失败')
    }
  } catch (error: any) {
    // 上传异常时同样清空任务状态
    currentTask.value = null
    packageFile.value = null
    sha256File.value = null
    uploadProgress.value = 0
    ElMessage.error(`上传失败: ${error.message}`)
  } finally {
    uploading.value = false
  }
}

// 开始更新
const startUpdate = async () => {
  if (!currentTask.value?.taskId) return

  updating.value = true
  try {
    const response = await startUpdateTask({
      taskId: currentTask.value.taskId,
      appType: currentTask.value.appType,  // 使用上传时保存的 appType
      daemon: form.value.daemon
    })

    if (response.success) {
      ElMessage.success('更新已启动，请注意观察进度')
      if (currentTask.value) {
        currentTask.value.state = 'PREPARING'
      }
      // 记录 WebSocket 连接状态
      wasConnected.value = wsStore.isConnected
    } else {
      ElMessage.error(response.message || '启动失败')
      updating.value = false
    }
  } catch (error: any) {
    ElMessage.error(`启动失败: ${error.message}`)
    updating.value = false
  }
}

// 取消更新
const cancelUpdate = async () => {
  if (!currentTask.value?.taskId) return

  try {
    const response = await cancelUpdateTask(currentTask.value.taskId)
    if (response.success) {
      ElMessage.info('更新已取消')
    }
  } catch (error: any) {
    ElMessage.error(`取消失败: ${error.message}`)
  }
}

// WebSocket 消息处理
const handleWebSocketMessage = (data: any) => {
  try {
    const message = typeof data === 'string' ? JSON.parse(data) : data

    if (message.type === 'update_status' && message.data) {
      console.log('[OTA] WebSocket 收到更新状态:', message.data)
      updateTaskStatus(message.data)
    }
  } catch (error) {
    console.error('WebSocket message parse error:', error)
  }
}

// 更新任务状态（统一处理）
const updateTaskStatus = (status: UpdateStatus) => {
  if (!currentTask.value || currentTask.value.taskId === status.taskId) {
    currentTask.value = status

    // 更新完成时处理
    if (isUpdateComplete(status.state)) {
      updating.value = false
      stopPolling()

      // 显示完成通知
      if (status.state === 'SUCCESS') {
        ElMessage.success({
          message: `更新成功！用时 ${status.duration || 0} 秒`,
          duration: 5000
        })
      } else if (status.state === 'FAILED') {
        ElMessage.error({
          message: `更新失败：${status.errorMsg || '未知错误'}`,
          duration: 0 // 不自动关闭
        })
      }
    }
  }
}

// 判断更新是否完成
const isUpdateComplete = (state: string): boolean => {
  return ['SUCCESS', 'FAILED', 'ROLLBACK_SUCCESS', 'ROLLBACK_FAILED', 'CANCELLED'].includes(state)
}

// 开始 HTTP 轮询
const startPolling = () => {
  if (pollingInterval.value) return

  console.log('[OTA] 启动 HTTP 轮询模式')
  pollingActive.value = true
  isWebSocketMode.value = false
  reconnectAttempts.value = 0

  pollingInterval.value = window.setInterval(async () => {
    reconnectAttempts.value++
    console.log(`[OTA] HTTP 轮询尝试 ${reconnectAttempts.value}/${maxReconnectAttempts.value}`)

    try {
      // 尝试获取当前状态
      const response = await getCurrentStatus()

      if (response.success && response.status) {
        console.log('[OTA] HTTP 轮询获取到状态:', response.status)
        updateTaskStatus(response.status)

        // 如果更新已完成，尝试重新连接 WebSocket
        if (isUpdateComplete(response.status.state)) {
          console.log('[OTA] 更新完成，尝试恢复 WebSocket 连接')
          await attemptReconnectWebSocket()
        }
      }
    } catch (error: any) {
      console.log(`[OTA] HTTP 轮询失败 (${reconnectAttempts.value}/${maxReconnectAttempts.value}):`, error.message)
      
      // 如果是更新期间的连接失败，继续轮询
      if (updating.value && reconnectAttempts.value < maxReconnectAttempts.value) {
        // 继续轮询
        return
      }
    }

    // 超时处理
    if (reconnectAttempts.value >= maxReconnectAttempts.value) {
      console.error('[OTA] HTTP 轮询超时')
      stopPolling()
      ElMessage.error({
        message: '更新状态查询超时，请手动刷新页面或检查设备状态',
        duration: 0
      })
    }
  }, 5000) // 每 5 秒轮询一次
}

// 停止 HTTP 轮询
const stopPolling = () => {
  if (pollingInterval.value) {
    clearInterval(pollingInterval.value)
    pollingInterval.value = null
  }
  pollingActive.value = false
  reconnectAttempts.value = 0
  console.log('[OTA] HTTP 轮询已停止')
}

// 尝试重新连接 WebSocket
const attemptReconnectWebSocket = async () => {
  if (wsStore.isConnected) {
    console.log('[OTA] WebSocket 已连接')
    isWebSocketMode.value = true
    stopPolling()
    return true
  }

  try {
    console.log('[OTA] 尝试重新连接 WebSocket...')
    await wsStore.connect()
    
    if (wsStore.isConnected) {
      console.log('[OTA] WebSocket 重连成功')
      ElMessage.success('连接已恢复')
      isWebSocketMode.value = true
      stopPolling()
      return true
    }
  } catch (error) {
    console.error('[OTA] WebSocket 重连失败:', error)
  }
  
  return false
}

// 监听 WebSocket 连接状态变化
watch(() => wsStore.isConnected, (connected, wasConnectedBefore) => {
  console.log('[OTA] WebSocket 连接状态变化:', { connected, wasConnectedBefore, updating: updating.value })

  // WebSocket 断开，且正在更新
  if (!connected && wasConnectedBefore && updating.value) {
    console.log('[OTA] WebSocket 断开，切换到 HTTP 轮询模式')
    ElMessage.warning('连接中断，已切换到轮询模式')
    startPolling()
  }

  // WebSocket 重新连接，停止轮询
  if (connected && !wasConnectedBefore && pollingActive.value) {
    console.log('[OTA] WebSocket 重新连接，停止 HTTP 轮询')
    ElMessage.success('连接已恢复')
    stopPolling()
    isWebSocketMode.value = true
  }
})

// 生命周期
onMounted(async () => {
  console.log('[OTA] 组件挂载')
  
  // 初始化 UUID 并连接
  if (!wsStore.isConnected) {
    await wsStore.initUUID()
    await wsStore.connect()
  }

  // 注册 WebSocket 消息监听
  wsStore.onMessage('/robot/video', handleWebSocketMessage)

  console.log('[OTA] WebSocket 初始状态:', wsStore.isConnected)
})

onUnmounted(() => {
  console.log('[OTA] 组件卸载')
  
  // 移除消息监听
  wsStore.offMessage('/robot/video', handleWebSocketMessage)
  
  // 停止轮询
  stopPolling()
})
</script>

<style scoped>
.ota-update {
  padding: 20px;
}

.card-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
}

.header-status {
  display: flex;
  align-items: center;
}

.app-type-hint {
  margin-top: 8px;
}

.upload-progress {
  margin-top: 20px;
  padding: 15px;
  background-color: #f5f7fa;
  border-radius: 4px;
}

.upload-tip {
  margin-top: 10px;
  text-align: center;
}

.progress-text {
  font-size: 14px;
  font-weight: 600;
  color: #303133;
}

.button-group {
  margin-top: 20px;
  display: flex;
  gap: 10px;
}

.progress-section {
  margin-top: 30px;
}

.progress-section h3 {
  margin-bottom: 15px;
  color: #303133;
}

.status-info {
  margin-top: 20px;
}

/* ===========================
   统一工业风格按钮样式
   =========================== */

/* 统一按钮尺寸 */
.button-group :deep(.el-button) {
  height: 40px !important;
  font-size: 14px !important;
}

/* 上传按钮（统一蓝色风格） */
.button-group :deep(.el-button--primary) {
  background: linear-gradient(145deg, #34495e 0%, #2c3e50 100%);
  border: 1px solid #3498db;
  color: #ecf0f1;
  font-weight: 600;
  transition: all 0.2s ease;
}

.button-group :deep(.el-button--primary:hover:not(:disabled)) {
  background: linear-gradient(145deg, #3498db 0%, #2980b9 100%);
  transform: translateY(-1px);
  box-shadow: 0 4px 12px rgba(52, 152, 219, 0.4);
}

.button-group :deep(.el-button--primary:active:not(:disabled)) {
  transform: translateY(0);
  box-shadow: 0 2px 6px rgba(52, 152, 219, 0.3);
}

.button-group :deep(.el-button--primary.is-loading) {
  background: linear-gradient(145deg, #34495e 0%, #2c3e50 100%);
  opacity: 0.6;
}

/* 开始更新按钮（统一蓝色风格） */
.button-group :deep(.el-button--success) {
  background: linear-gradient(145deg, #34495e 0%, #2c3e50 100%);
  border: 1px solid #3498db;
  color: #ecf0f1;
  font-weight: 600;
  transition: all 0.2s ease;
}

.button-group :deep(.el-button--success:hover:not(:disabled)) {
  background: linear-gradient(145deg, #3498db 0%, #2980b9 100%);
  transform: translateY(-1px);
  box-shadow: 0 4px 12px rgba(52, 152, 219, 0.4);
}

.button-group :deep(.el-button--success:active:not(:disabled)) {
  transform: translateY(0);
  box-shadow: 0 2px 6px rgba(52, 152, 219, 0.3);
}

.button-group :deep(.el-button--success.is-loading) {
  background: linear-gradient(145deg, #34495e 0%, #2c3e50 100%);
  opacity: 0.6;
}

/* 取消按钮（红色警告） */
.button-group :deep(.el-button--danger) {
  background: linear-gradient(145deg, #34495e 0%, #2c3e50 100%);
  border: 1px solid #e74c3c;
  color: #ecf0f1;
  font-weight: 600;
  transition: all 0.2s ease;
}

.button-group :deep(.el-button--danger:hover:not(:disabled)) {
  background: linear-gradient(145deg, #e74c3c 0%, #c0392b 100%);
  border-color: #e74c3c;
  transform: translateY(-1px);
  box-shadow: 0 4px 12px rgba(231, 76, 60, 0.4);
}

.button-group :deep(.el-button--danger:active:not(:disabled)) {
  transform: translateY(0);
  box-shadow: 0 2px 6px rgba(231, 76, 60, 0.3);
}
</style>

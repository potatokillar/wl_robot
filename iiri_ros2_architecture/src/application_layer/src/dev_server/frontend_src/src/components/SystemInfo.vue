<template>
    <el-card header="系统信息" class="system-info">
        <el-row :gutter="20">
            <!-- 连接信息 -->
            <el-col :span="8">
                <div class="info-section">
                    <h4>连接状态</h4>
                    <el-descriptions :column="1" size="small">
                        <el-descriptions-item label="服务器">
                            {{ wsStore.config.ip }}:{{ wsStore.config.port }}
                        </el-descriptions-item>
                        <el-descriptions-item label="设备UUID">
                            {{ wsStore.config.uuid }}
                        </el-descriptions-item>
                        <el-descriptions-item label="连接数">
                            {{ wsStore.connectionCount() }}
                        </el-descriptions-item>
                        <el-descriptions-item label="状态">
                            <el-tag :type="connectionStatusType">
                                {{ connectionStatusText }}
                            </el-tag>
                        </el-descriptions-item>
                    </el-descriptions>
                </div>
            </el-col>

            <!-- 机器人状态 -->
            <el-col :span="8">
                <div class="info-section">
                    <h4>机器人状态</h4>
                    <el-descriptions :column="1" size="small">
                        <el-descriptions-item label="运行状态">
                            <el-tag :type="robotStatusType">
                                {{ robotStore.status.runState }}
                            </el-tag>
                        </el-descriptions-item>
                        <el-descriptions-item label="电池电量">
                            <el-progress :percentage="robotStore.status.battery" :color="batteryColor"
                                :stroke-width="6" />
                        </el-descriptions-item>
                        <el-descriptions-item label="位置">
                            X: {{ robotStore.status.position.x.toFixed(2) }}m,
                            Y: {{ robotStore.status.position.y.toFixed(2) }}m
                        </el-descriptions-item>
                        <el-descriptions-item label="姿态">
                            Yaw: {{ (robotStore.status.orientation.yaw * 180 / Math.PI).toFixed(1) }}°
                        </el-descriptions-item>
                    </el-descriptions>
                </div>
            </el-col>

            <!-- 系统性能 -->
            <el-col :span="8">
                <div class="info-section">
                    <h4>系统性能</h4>
                    <el-descriptions :column="1" size="small">
                        <el-descriptions-item label="视频帧率">
                            {{ videoStats.fps }} FPS
                        </el-descriptions-item>
                        <el-descriptions-item label="网络延迟">
                            {{ videoStats.latency }}ms
                        </el-descriptions-item>
                        <el-descriptions-item label="数据传输">
                            ↓{{ formatDataRate(networkStats.downloadRate) }}/s
                            ↑{{ formatDataRate(networkStats.uploadRate) }}/s
                        </el-descriptions-item>
                        <el-descriptions-item label="CPU使用率">
                            <el-progress :percentage="systemStats.cpuUsage" :stroke-width="6" :color="cpuColor" />
                        </el-descriptions-item>
                    </el-descriptions>
                </div>
            </el-col>
        </el-row>

        <!-- 实时日志 -->
        <el-divider />
        <div class="log-section">
            <h4>
                实时日志
                <el-button size="small" @click="clearLogs" style="margin-left: 10px">
                    清空
                </el-button>
            </h4>
            <div class="log-container" ref="logContainer">
                <div v-for="(log, index) in logs" :key="index" :class="['log-item', `log-${log.level}`]">
                    <span class="log-time">{{ formatTime(log.timestamp) }}</span>
                    <span class="log-level">[{{ log.level.toUpperCase() }}]</span>
                    <span class="log-message">{{ log.message }}</span>
                </div>
                <div v-if="logs.length === 0" class="no-logs">
                    暂无日志信息
                </div>
            </div>
        </div>
    </el-card>
</template>

<script setup lang="ts">
import { ref, computed, onMounted, onUnmounted, nextTick } from 'vue'
import { useWebSocketStore } from '../stores/websocket'
import { useRobotStore } from '../stores/robot'

const wsStore = useWebSocketStore()
const robotStore = useRobotStore()

// 模板引用
const logContainer = ref<HTMLDivElement>()

// 统计数据
const videoStats = ref({
    fps: 0,
    latency: 0
})

const networkStats = ref({
    downloadRate: 0,
    uploadRate: 0,
    totalDownload: 0,
    totalUpload: 0
})

const systemStats = ref({
    cpuUsage: 0,
    memoryUsage: 0
})

// 日志
interface LogItem {
    timestamp: Date
    level: 'info' | 'warn' | 'error' | 'debug'
    message: string
}

const logs = ref<LogItem[]>([])
const maxLogs = 100

// 计算属性
const connectionStatusType = computed(() => {
    if (wsStore.isConnected) return 'success'
    if (wsStore.isConnecting) return 'warning'
    return 'danger'
})

const connectionStatusText = computed(() => {
    if (wsStore.isConnected) return '已连接'
    if (wsStore.isConnecting) return '连接中'
    return '未连接'
})

const robotStatusType = computed(() => {
    switch (robotStore.status.runState) {
        case 'walk': return 'success'
        case 'stand': return 'primary'
        case 'lie': return 'warning'
        default: return 'info'
    }
})

const batteryColor = computed(() => {
    const battery = robotStore.status.battery
    if (battery > 60) return '#67c23a'
    if (battery > 30) return '#e6a23c'
    return '#f56c6c'
})

const cpuColor = computed(() => {
    const cpu = systemStats.value.cpuUsage
    if (cpu < 70) return '#67c23a'
    if (cpu < 90) return '#e6a23c'
    return '#f56c6c'
})

onMounted(() => {
    startMonitoring()
    addLog('info', '系统监控已启动')
})

onUnmounted(() => {
    stopMonitoring()
})

let monitoringInterval: number | null = null

/**
 * 开始监控
 */
function startMonitoring() {
    monitoringInterval = setInterval(() => {
        updateStats()
    }, 1000)
}

/**
 * 停止监控
 */
function stopMonitoring() {
    if (monitoringInterval) {
        clearInterval(monitoringInterval)
        monitoringInterval = null
    }
}

/**
 * 更新统计数据
 */
function updateStats() {
    // 模拟数据更新（实际项目中应该从相应的store获取）
    videoStats.value.fps = Math.floor(Math.random() * 30) + 20
    videoStats.value.latency = Math.floor(Math.random() * 50) + 20

    networkStats.value.downloadRate = Math.floor(Math.random() * 1000000) + 500000
    networkStats.value.uploadRate = Math.floor(Math.random() * 100000) + 50000

    systemStats.value.cpuUsage = Math.floor(Math.random() * 40) + 20
    systemStats.value.memoryUsage = Math.floor(Math.random() * 30) + 40

    // 随机添加日志
    if (Math.random() > 0.8) {
        const messages = [
            '收到视频数据包',
            '发送控制命令',
            '心跳检测正常',
            '音频数据传输',
            '机器人状态更新'
        ]
        addLog('info', messages[Math.floor(Math.random() * messages.length)])
    }
}

/**
 * 添加日志
 */
function addLog(level: LogItem['level'], message: string) {
    logs.value.push({
        timestamp: new Date(),
        level,
        message
    })

    // 限制日志数量
    if (logs.value.length > maxLogs) {
        logs.value = logs.value.slice(-maxLogs)
    }

    // 自动滚动到底部
    nextTick(() => {
        if (logContainer.value) {
            logContainer.value.scrollTop = logContainer.value.scrollHeight
        }
    })
}

/**
 * 清空日志
 */
function clearLogs() {
    logs.value = []
    addLog('info', '日志已清空')
}

/**
 * 格式化时间
 */
function formatTime(date: Date): string {
    return date.toLocaleTimeString()
}

/**
 * 格式化数据传输速率
 */
function formatDataRate(bytes: number): string {
    if (bytes < 1024) return `${bytes}B`
    if (bytes < 1024 * 1024) return `${(bytes / 1024).toFixed(1)}KB`
    return `${(bytes / 1024 / 1024).toFixed(1)}MB`
}

// 暴露方法给外部使用
defineExpose({
    addLog
})
</script>

<style scoped>
.system-info {
    height: 100%;
}

.info-section h4 {
    margin: 0 0 15px 0;
    color: #333;
    font-size: 14px;
}

.log-section {
    margin-top: 20px;
}

.log-section h4 {
    margin: 0 0 15px 0;
    color: #333;
    font-size: 14px;
    display: flex;
    align-items: center;
}

.log-container {
    height: 120px;
    overflow-y: auto;
    background-color: #f8f9fa;
    border: 1px solid #e9ecef;
    border-radius: 4px;
    padding: 10px;
    font-family: 'Consolas', 'Monaco', monospace;
    font-size: 12px;
}

.log-item {
    margin-bottom: 5px;
    line-height: 1.4;
}

.log-time {
    color: #6c757d;
    margin-right: 8px;
}

.log-level {
    margin-right: 8px;
    font-weight: bold;
}

.log-info .log-level {
    color: #17a2b8;
}

.log-warn .log-level {
    color: #ffc107;
}

.log-error .log-level {
    color: #dc3545;
}

.log-debug .log-level {
    color: #6f42c1;
}

.log-message {
    color: #333;
}

.no-logs {
    color: #6c757d;
    text-align: center;
    padding: 20px 0;
}

/* 自定义滚动条样式 */
.log-container::-webkit-scrollbar {
    width: 6px;
}

.log-container::-webkit-scrollbar-track {
    background: #f1f1f1;
    border-radius: 3px;
}

.log-container::-webkit-scrollbar-thumb {
    background: #c1c1c1;
    border-radius: 3px;
}

.log-container::-webkit-scrollbar-thumb:hover {
    background: #a8a8a8;
}
</style>
<template>
    <el-card class="connection-panel" shadow="hover">
        <!-- Header with device name -->
        <template #header>
            <div class="panel-header">
                <div class="header-left">
                    <h3 class="device-name">
                        机器人状态
                        <el-tooltip content="刷新 UUID" placement="top">
                            <el-button
                                :icon="Refresh"
                                :loading="uuidLoading"
                                circle
                                size="small"
                                @click="refreshUUID"
                                class="refresh-button"
                            />
                        </el-tooltip>
                    </h3>
                </div>
            </div>
        </template>

        <!-- Connection information -->
        <div class="connection-info">
            <!-- IP Address -->
            <div class="info-section">
                <div class="info-item">
                    <label class="info-label">IP 地址</label>
                    <div class="info-value-group">
                        <el-input
                            v-model="wsStore.config.ip"
                            class="info-input"
                            placeholder="请输入IP地址"
                            :disabled="wsStore.isConnected || wsStore.isConnecting"
                        />
                        <copy-button :value="wsStore.config.ip" size="small" />
                    </div>
                </div>

                <!-- Port -->
                <div class="info-item info-item--inline">
                    <label class="info-label">端口</label>
                    <el-tag
                        :type="wsStore.isConnected ? 'success' : 'info'"
                        size="large"
                        class="port-tag"
                    >
                        {{ wsStore.config.port }}
                    </el-tag>
                </div>
            </div>

            <!-- Advanced details (collapsible) -->
            <el-collapse class="advanced-details" v-model="activeCollapse">
                <el-collapse-item title="技术详情" name="technical">
                    <div class="detail-grid">
                        <div class="detail-item">
                            <label class="detail-label">设备 UUID</label>
                            <div class="detail-value-group">
                                <el-tooltip :content="wsStore.config.uuid" placement="top">
                                    <code class="detail-value detail-value--uuid">
                                        {{ truncatedUuid }}
                                    </code>
                                </el-tooltip>
                                <copy-button :value="wsStore.config.uuid" size="small" />
                            </div>
                        </div>

                        <div class="detail-item">
                            <label class="detail-label">连接协议</label>
                            <span class="detail-value">WebSocket (WS)</span>
                        </div>

                        <div class="detail-item">
                            <label class="detail-label">连接数</label>
                            <span class="detail-value">{{ wsStore.connectionCount() }}</span>
                        </div>

                        <!-- 连接节点列表 - 工业风格 LED 面板 -->
                        <div class="detail-item detail-item--full-width" v-if="wsStore.isConnected && connectionDetails.length > 0">
                            <label class="detail-label">连接节点</label>
                            <div class="connection-nodes">
                                <div v-for="(connections, category) in groupedConnections" :key="category" class="node-category">
                                    <!-- 分类标题 - 金属标签贴效果 -->
                                    <div class="category-header">
                                        <div class="category-label">{{ category }}</div>
                                    </div>
                                    <!-- 节点网格 -->
                                    <div class="node-grid">
                                        <div
                                            v-for="conn in connections"
                                            :key="conn.channel"
                                            :class="[
                                                'node-card',
                                                `node-card--${conn.status}`
                                            ]"
                                        >
                                            <!-- LED 指示灯 -->
                                            <div :class="['led-indicator', `led-indicator--${conn.status}`]">
                                                <div class="led-light"></div>
                                            </div>
                                            <!-- 节点信息 -->
                                            <div class="node-content">
                                                <el-icon class="node-icon">
                                                    <component :is="iconMap[conn.icon]" />
                                                </el-icon>
                                                <span class="node-name">{{ conn.displayName }}</span>
                                            </div>
                                        </div>
                                    </div>
                                </div>
                            </div>
                        </div>
                    </div>
                </el-collapse-item>
            </el-collapse>

            <!-- Action buttons -->
            <div class="action-section">
                <el-button
                    v-if="!wsStore.isConnected"
                    type="primary"
                    :loading="wsStore.isConnecting"
                    :disabled="!canConnect"
                    @click="handleConnect"
                    class="action-button"
                    size="large"
                >
                    {{ wsStore.isConnecting ? '连接中...' : '连接' }}
                </el-button>

                <el-button
                    v-else
                    type="danger"
                    @click="handleDisconnect"
                    class="action-button"
                    size="large"
                >
                    断开连接
                </el-button>

                <el-alert
                    v-if="wsStore.connectionError"
                    type="error"
                    :title="wsStore.connectionError"
                    :closable="false"
                    show-icon
                    class="error-alert"
                />
            </div>
        </div>

        <!-- Footer with metrics -->
        <template #footer v-if="wsStore.isConnected">
            <div class="panel-footer">
                <div class="metric">
                    <el-icon><Timer /></el-icon>
                    <span>连接时长: {{ connectionTime }}</span>
                </div>
                <div class="metric">
                    <el-icon><Connection /></el-icon>
                    <span>活跃连接: {{ wsStore.connectionCount() }}</span>
                </div>
            </div>
        </template>
    </el-card>
</template>

<script setup lang="ts">
import { computed, ref, onMounted, onUnmounted } from 'vue'
import { useWebSocketStore } from '../stores/websocket'
import { ElMessage } from 'element-plus'
import {
    Refresh,
    Connection,
    Timer,
    VideoCamera,
    Microphone,
    Headset,
    Camera,
    Aim,
    Setting,
    ChatLineSquare
} from '@element-plus/icons-vue'
import CopyButton from './CopyButton.vue'

const wsStore = useWebSocketStore()

// 图标映射
const iconMap: Record<string, any> = {
    'VideoCamera': VideoCamera,
    'Microphone': Microphone,
    'Headset': Headset,
    'Camera': Camera,
    'Aim': Aim,
    'Setting': Setting,
    'ChatLineSquare': ChatLineSquare,
    'Connection': Connection
}

// UUID 加载状态
const uuidLoading = ref(false)

// 连接时间显示
const connectionTime = ref('')
const connectionStartTime = ref<Date | null>(null)
let timeInterval: number | null = null

// 高级详情折叠状态
const activeCollapse = ref<string[]>([])

// 计算属性
const canConnect = computed(() => {
    return wsStore.config.ip.trim() !== '' &&
        wsStore.config.uuid.trim() !== '' &&
        wsStore.config.port > 0
})

const connectionStatus = computed(() => {
    if (wsStore.isConnected) return 'connected'
    if (wsStore.isConnecting) return 'connecting'
    return 'disconnected'
})

const connectionTimeText = computed(() => {
    if (!connectionStartTime.value) return '刚刚连接'

    const now = new Date()
    const diff = now.getTime() - connectionStartTime.value.getTime()
    const minutes = Math.floor(diff / 60000)

    if (minutes < 1) return '刚刚连接'
    if (minutes === 1) return '1 分钟前连接'
    if (minutes < 60) return `${minutes} 分钟前连接`

    const hours = Math.floor(minutes / 60)
    return hours === 1 ? '1 小时前连接' : `${hours} 小时前连接`
})

const truncatedUuid = computed(() => {
    const uuid = wsStore.config.uuid
    if (!uuid || uuid.length <= 16) return uuid
    return `${uuid.substring(0, 8)}...${uuid.substring(uuid.length - 4)}`
})

// 获取连接详情
const connectionDetails = computed(() => {
    return wsStore.getConnectionDetails()
})

// 按分类分组连接
const groupedConnections = computed(() => {
    const groups: Record<string, typeof connectionDetails.value> = {}
    connectionDetails.value.forEach(conn => {
        if (!groups[conn.category]) {
            groups[conn.category] = []
        }
        groups[conn.category].push(conn)
    })
    return groups
})

// 方法
async function handleConnect() {
    try {
        await wsStore.connect()
        ElMessage.success('连接成功')
        connectionStartTime.value = new Date()
        startTimeCounter()
    } catch (error) {
        ElMessage.error(`连接失败: ${error}`)
    }
}

function handleDisconnect() {
    wsStore.disconnect()
    ElMessage.info('已断开连接')
    connectionStartTime.value = null
    stopTimeCounter()
}

function startTimeCounter() {
    const startTime = Date.now()
    timeInterval = setInterval(() => {
        const elapsed = Math.floor((Date.now() - startTime) / 1000)
        const hours = Math.floor(elapsed / 3600)
        const minutes = Math.floor((elapsed % 3600) / 60)
        const seconds = elapsed % 60
        connectionTime.value = `${hours.toString().padStart(2, '0')}:${minutes.toString().padStart(2, '0')}:${seconds.toString().padStart(2, '0')}`
    }, 1000)
}

function stopTimeCounter() {
    if (timeInterval) {
        clearInterval(timeInterval)
        timeInterval = null
    }
    connectionTime.value = ''
}

// 刷新 UUID
async function refreshUUID() {
    uuidLoading.value = true
    try {
        await wsStore.initUUID()
        ElMessage.success('UUID 获取成功')
    } catch (error) {
        ElMessage.error('UUID 获取失败')
    } finally {
        uuidLoading.value = false
    }
}

onMounted(async () => {
    // 页面加载时自动获取 UUID
    await refreshUUID()

    if (wsStore.isConnected) {
        connectionStartTime.value = new Date()
        startTimeCounter()
    }
})

onUnmounted(() => {
    stopTimeCounter()
})
</script>

<style scoped>
/* ===== CSS Variables ===== */
.connection-panel {
    --card-padding: 24px;
    --section-gap: 20px;
    --item-gap: 16px;
    --label-value-gap: 6px;

    /* Colors - 工业风格配色 */
    --status-success: #52c41a;
    --status-warning: #faad14;
    --status-danger: #ff4d4f;
    --status-info: #1890ff;

    /* Monospace font for technical data */
    --font-mono: 'Monaco', 'Consolas', 'Courier New', monospace;

    /* 工业风格卡片样式 */
    background-color: #2c3e50;
    border: 1px solid #34495e;
    border-radius: 8px;
    box-shadow: 0 2px 8px rgba(0, 0, 0, 0.3);
    transition: all 0.3s cubic-bezier(0.4, 0, 0.2, 1);
}

.connection-panel:hover {
    transform: translateY(-2px);
    box-shadow: 0 4px 12px rgba(0, 0, 0, 0.5);
}

.connection-panel :deep(.el-card__header) {
    background-color: #34495e;
    color: #ecf0f1;
    border-bottom: 2px solid #3498db;
    font-weight: 600;
    padding: 12px 16px;
}

.connection-panel :deep(.el-card__body) {
    padding: 16px;
}

.connection-panel :deep(.el-card__footer) {
    background-color: #2c3e50;
    border-top: 1px solid #34495e;
    padding: 12px 16px;
}

/* ===== Header ===== */
.panel-header {
    display: flex;
    align-items: center;
    justify-content: space-between;
    gap: 16px;
}

.header-left {
    flex: 1;
    min-width: 0;
}

.status-indicator {
    display: flex;
    align-items: center;
    gap: 12px;
    margin-bottom: 6px;
}

.status-dot {
    width: 10px;
    height: 10px;
    border-radius: 50%;
    flex-shrink: 0;
    position: relative;
}

.status-dot--connected {
    background: var(--status-success);
}

.status-dot--connected::before {
    content: '';
    position: absolute;
    inset: -4px;
    border-radius: 50%;
    background: var(--status-success);
    opacity: 0.3;
    animation: pulse 2s infinite;
}

@keyframes pulse {
    0%, 100% {
        transform: scale(1);
        opacity: 0.3;
    }
    50% {
        transform: scale(1.5);
        opacity: 0;
    }
}

.status-dot--connecting {
    background: var(--status-warning);
    animation: blink 1s infinite;
}

@keyframes blink {
    0%, 50% {
        opacity: 1;
    }
    25%, 75% {
        opacity: 0.3;
    }
}

.status-dot--disconnected {
    background: var(--status-danger);
}

.device-name {
    font-size: 18px;
    font-weight: 600;
    margin: 0;
    color: #ffffff;
    display: flex;
    align-items: center;
    gap: 48px;
}

.refresh-button {
    width: 28px !important;
    height: 28px !important;
    min-width: 28px !important;
    max-width: 28px !important;
    min-height: 28px !important;
    max-height: 28px !important;
    padding: 0 !important;
    font-size: 14px !important;
    flex-shrink: 0 !important;
    background: rgba(64, 158, 255, 0.15) !important;
    border: 1px solid rgba(64, 158, 255, 0.4) !important;
    color: #409eff !important;
    transition: all 0.3s ease !important;
}

.refresh-button:hover {
    background: rgba(64, 158, 255, 0.25) !important;
    border-color: #409eff !important;
    box-shadow: 0 0 8px rgba(64, 158, 255, 0.3) !important;
}

.status-time {
    font-size: 12px;
    color: var(--el-text-color-secondary);
}

.status-time--connecting {
    color: var(--status-warning);
}

.status-time--disconnected {
    color: var(--status-danger);
}

.header-actions {
    display: flex;
    gap: 8px;
}

/* ===== Connection Info ===== */
.connection-info {
    display: flex;
    flex-direction: column;
    gap: var(--section-gap);
}

.info-section {
    display: flex;
    flex-direction: column;
    gap: var(--item-gap);
}

.info-item {
    display: flex;
    flex-direction: column;
    gap: var(--label-value-gap);
}

.info-item--inline {
    flex-direction: row;
    align-items: center;
    gap: 12px;
}

.info-label {
    font-size: 12px;
    font-weight: 600;
    color: #9ca3af;
    text-transform: uppercase;
    letter-spacing: 0.5px;
}

.info-value-group {
    display: flex;
    align-items: center;
    gap: 8px;
}

.info-input {
    flex: 1;
}

/* 工业风格输入框 */
.info-input :deep(.el-input__wrapper) {
    font-family: var(--font-mono);
    font-size: 14px;
    background: linear-gradient(145deg, #1a1d23 0%, #0f1115 100%);
    border: 1px solid #34495e;
    box-shadow: inset 0 2px 4px rgba(0, 0, 0, 0.3);
}

.info-input :deep(.el-input__wrapper:hover) {
    border-color: #3498db;
}

.info-input :deep(.el-input__wrapper.is-focus) {
    border-color: #3498db;
    box-shadow: 0 0 8px rgba(52, 152, 219, 0.4);
}

.info-input :deep(.el-input__inner) {
    color: #ecf0f1;
}

/* ===== Port Display ===== */
.port-display {
    display: flex;
    align-items: center;
    justify-content: center;
    gap: 12px;
}

.port-tag {
    font-size: 16px;
    font-weight: 600;
    padding: 0 20px;
    border-radius: 6px;
    height: 38px;
    line-height: 38px;
    display: inline-block;
    text-align: center;
    background-color: #000000 !important;
    color: #ecf0f1 !important;
    border: 1px solid #34495e !important;
    vertical-align: middle;
}

.port-protocol {
    font-size: 13px;
    color: var(--el-text-color-secondary);
}

/* ===== Advanced Details ===== */
.advanced-details {
    margin-top: 4px;
    border: none;
    background: transparent;
}

.advanced-details :deep(.el-collapse-item__header) {
    background: linear-gradient(145deg, #1a1d23 0%, #0f1115 100%);
    border-radius: 6px;
    padding: 10px 16px;
    font-size: 13px;
    font-weight: 600;
    color: #9ca3af;
    border: 1px solid #34495e;
    transition: all 0.2s;
    text-transform: uppercase;
    letter-spacing: 0.5px;
}

.advanced-details :deep(.el-collapse-item__header:hover) {
    background: linear-gradient(145deg, #2a2d33 0%, #1f2125 100%);
    border-color: #3498db;
}

.advanced-details :deep(.el-collapse-item__wrap) {
    background: transparent;
    border: none;
}

.detail-grid {
    display: grid;
    grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
    gap: 16px;
    padding-top: 16px;
}

.detail-item {
    display: flex;
    flex-direction: column;
    gap: 4px;
}

.detail-label {
    font-size: 11px;
    font-weight: 600;
    color: #9ca3af;
    text-transform: uppercase;
    letter-spacing: 0.5px;
}

.detail-value-group {
    display: flex;
    align-items: center;
    gap: 8px;
    padding: 8px 12px;
    background: linear-gradient(145deg, #1a1d23 0%, #0f1115 100%);
    border-radius: 4px;
    border: 1px solid #34495e;
    box-shadow: inset 0 2px 4px rgba(0, 0, 0, 0.3);
}

.detail-value {
    font-size: 13px;
    color: #ecf0f1;
}

.detail-value--uuid {
    font-family: var(--font-mono);
    flex: 1;
    overflow: hidden;
    text-overflow: ellipsis;
    white-space: nowrap;
}

/* ===== 连接节点列表 - 工业风格 LED 面板 ===== */
.detail-item--full-width {
    grid-column: 1 / -1;
}

.connection-nodes {
    display: flex;
    flex-direction: column;
    gap: 20px;
    padding: 16px;
    background: linear-gradient(145deg, #0a0c0f 0%, #14171a 100%);
    border-radius: 8px;
    border: 1px solid #2c3e50;
    box-shadow: inset 0 2px 4px rgba(0, 0, 0, 0.5);
}

.node-category {
    display: flex;
    flex-direction: column;
    gap: 12px;
}

/* 分类标题 - 金属标签贴效果 */
.category-header {
    position: relative;
    margin-bottom: 4px;
}

.category-label {
    display: inline-block;
    font-size: 10px;
    font-weight: 700;
    color: #3498db;
    text-transform: uppercase;
    letter-spacing: 1.2px;
    padding: 6px 16px;
    background: linear-gradient(145deg, #1c2127 0%, #0f1419 100%);
    border: 1px solid #3498db;
    border-radius: 3px 3px 0 0;
    box-shadow:
        0 2px 4px rgba(0, 0, 0, 0.3),
        inset 0 1px 0 rgba(52, 152, 219, 0.2);
    position: relative;
}

.category-label::before {
    content: '';
    position: absolute;
    left: 0;
    top: 0;
    right: 0;
    height: 1px;
    background: linear-gradient(90deg,
        transparent 0%,
        rgba(52, 152, 219, 0.6) 50%,
        transparent 100%);
}

/* 节点网格布局 */
.node-grid {
    display: grid;
    grid-template-columns: repeat(auto-fill, minmax(160px, 1fr));
    gap: 12px;
    padding: 4px;
}

/* 节点卡片 - SCADA 面板风格 */
.node-card {
    display: flex;
    align-items: center;
    gap: 12px;
    padding: 10px 14px;
    background: linear-gradient(145deg, #1c1f26 0%, #14171d 100%);
    border: 1px solid #34495e;
    border-radius: 4px;
    box-shadow:
        inset 0 1px 2px rgba(0, 0, 0, 0.4),
        0 1px 3px rgba(0, 0, 0, 0.2);
    transition: all 0.25s ease;
    cursor: default;
    position: relative;
    overflow: hidden;
}

/* 卡片悬停效果 - 微妙高光扫过 */
.node-card::before {
    content: '';
    position: absolute;
    top: 0;
    left: -100%;
    width: 100%;
    height: 100%;
    background: linear-gradient(90deg,
        transparent 0%,
        rgba(52, 152, 219, 0.1) 50%,
        transparent 100%);
    transition: left 0.5s ease;
}

.node-card:hover::before {
    left: 100%;
}

.node-card:hover {
    border-color: #3498db;
    box-shadow:
        inset 0 1px 2px rgba(0, 0, 0, 0.4),
        0 2px 8px rgba(52, 152, 219, 0.2);
}

/* LED 指示灯容器 */
.led-indicator {
    width: 14px;
    height: 14px;
    border-radius: 50%;
    background: #0a0c0f;
    border: 1px solid #1a1d23;
    box-shadow: inset 0 2px 3px rgba(0, 0, 0, 0.8);
    display: flex;
    align-items: center;
    justify-content: center;
    flex-shrink: 0;
}

/* LED 灯光 */
.led-light {
    width: 8px;
    height: 8px;
    border-radius: 50%;
    transition: all 0.3s ease;
}

/* 已连接状态 - 绿色发光 */
.led-indicator--connected .led-light {
    background: #52c41a;
    box-shadow:
        0 0 4px #52c41a,
        0 0 8px rgba(82, 196, 26, 0.6),
        0 0 12px rgba(82, 196, 26, 0.4),
        inset 0 -2px 4px rgba(0, 0, 0, 0.3);
}

/* 连接中状态 - 黄色脉冲 */
.led-indicator--connecting .led-light {
    background: #faad14;
    box-shadow:
        0 0 4px #faad14,
        0 0 8px rgba(250, 173, 20, 0.6),
        0 0 12px rgba(250, 173, 20, 0.4);
    animation: led-pulse 1.5s ease-in-out infinite;
}

@keyframes led-pulse {
    0%, 100% {
        opacity: 1;
        transform: scale(1);
    }
    50% {
        opacity: 0.4;
        transform: scale(0.85);
    }
}

/* 断开连接状态 - 暗红色 */
.led-indicator--disconnected .led-light {
    background: #3a3a3a;
    box-shadow:
        inset 0 1px 2px rgba(0, 0, 0, 0.8);
}

/* 节点内容 */
.node-content {
    display: flex;
    align-items: center;
    gap: 8px;
    flex: 1;
    min-width: 0;
}

.node-icon {
    font-size: 14px;
    color: #7f8c8d;
    flex-shrink: 0;
}

.node-card--connected .node-icon {
    color: #52c41a;
}

.node-card--connecting .node-icon {
    color: #faad14;
}

.node-name {
    font-size: 12px;
    font-weight: 500;
    color: #bdc3c7;
    white-space: nowrap;
    overflow: hidden;
    text-overflow: ellipsis;
}

.node-card--connected .node-name {
    color: #ecf0f1;
}

/* ===== Action Section ===== */
.action-section {
    margin-top: 4px;
    padding-top: var(--section-gap);
    border-top: 1px solid #34495e;
    display: flex;
    flex-direction: column;
    gap: 12px;
}

/* 工业风格连接按钮 - 与机器人控制按钮一致 */
.action-button {
    width: 100% !important;
    height: 48px !important;
    min-width: unset !important;
    margin: 0 !important;
    padding: 0 !important;
    font-size: 15px !important;
    font-weight: 600;
    background: linear-gradient(145deg, #34495e 0%, #2c3e50 100%);
    border: 1px solid #3498db;
    color: #ecf0f1;
    transition: all 0.2s ease;
    display: flex !important;
    align-items: center !important;
    justify-content: center !important;
    box-sizing: border-box !important;
}

/* 连接按钮 Hover 效果 */
.connection-panel :deep(.el-button--primary):not(.is-loading):not(:disabled):hover {
    background: linear-gradient(145deg, #3498db 0%, #2980b9 100%) !important;
    transform: translateY(-1px);
    box-shadow: 0 4px 12px rgba(52, 152, 219, 0.4);
    border-color: #3498db !important;
}

/* 连接按钮 Active 效果 */
.connection-panel :deep(.el-button--primary):not(.is-loading):not(:disabled):active {
    transform: translateY(0);
    box-shadow: 0 2px 6px rgba(52, 152, 219, 0.3);
}

/* 断开连接按钮样式 */
.connection-panel :deep(.el-button--danger) {
    background: linear-gradient(145deg, #34495e 0%, #2c3e50 100%) !important;
    border: 1px solid #e74c3c !important;
    color: #ecf0f1 !important;
}

.connection-panel :deep(.el-button--danger):not(:disabled):hover {
    background: linear-gradient(145deg, #e74c3c 0%, #c0392b 100%) !important;
    border-color: #e74c3c !important;
    transform: translateY(-1px);
    box-shadow: 0 4px 12px rgba(231, 76, 60, 0.4);
}

.connection-panel :deep(.el-button--danger):not(:disabled):active {
    transform: translateY(0);
    box-shadow: 0 2px 6px rgba(231, 76, 60, 0.3);
}

/* Loading 状态 */
.connection-panel :deep(.el-button.is-loading) {
    background: linear-gradient(145deg, #34495e 0%, #2c3e50 100%) !important;
    opacity: 0.8;
}

/* Disabled 状态 */
.connection-panel :deep(.el-button:disabled) {
    opacity: 0.4;
    cursor: not-allowed;
    background: #2c3e50 !important;
    border-color: #34495e !important;
}

.error-alert {
    margin: 0;
}

/* ===== Footer ===== */
.panel-footer {
    display: flex;
    gap: 24px;
    padding-top: 4px;
}

.metric {
    display: flex;
    align-items: center;
    gap: 8px;
    font-size: 13px;
    color: #ecf0f1;
    padding: 8px 12px;
    background: linear-gradient(145deg, #34495e 0%, #2c3e50 100%);
    border-radius: 6px;
    border: 1px solid #3498db;
    flex: 1;
}

.metric .el-icon {
    font-size: 16px;
    color: #3498db;
}

/* ===== Responsive Design ===== */
@media (max-width: 768px) {
    .connection-panel {
        --card-padding: 16px;
        --section-gap: 16px;
        --item-gap: 12px;
    }

    .panel-header {
        flex-direction: column;
        align-items: flex-start;
    }

    .device-name {
        font-size: 16px;
    }

    .refresh-button {
        /* 手机端保持原样，不做缩放 */
    }

    .detail-grid {
        grid-template-columns: 1fr;
    }

    .panel-footer {
        flex-direction: column;
        gap: 12px;
    }
}
</style>

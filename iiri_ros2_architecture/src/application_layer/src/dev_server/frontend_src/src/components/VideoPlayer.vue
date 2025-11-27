<template>
    <div class="video-player">
        <div class="video-container" ref="videoContainer">
            <canvas ref="videoCanvas" class="video-canvas" :width="canvasWidth" :height="canvasHeight"></canvas>

            <!-- 视频覆盖层：状态标签和画质按钮 -->
            <div class="video-overlay">
                <!-- 左上角：状态标签 -->
                <div class="video-status">
                    <el-tag v-if="!wsStore.isConnected" type="danger" size="large">
                        未连接
                    </el-tag>
                    <el-tag v-else-if="!isReceivingVideo" type="warning" size="large">
                        等待视频流...
                    </el-tag>
                    <el-tag v-else type="success" size="large">
                        {{ videoStats.fps }} FPS
                    </el-tag>
                </div>

                <!-- 右上角：画质按钮（连接后显示） -->
                <div class="video-quality-btn" v-if="wsStore.isConnected">
                    <el-dropdown @command="handleQualityChange" trigger="click">
                        <el-button type="primary" size="small" round>
                            {{ currentQuality }}
                            <el-icon class="el-icon--right">
                                <ArrowDown />
                            </el-icon>
                        </el-button>
                        <template #dropdown>
                            <el-dropdown-menu>
                                <el-dropdown-item command="720p">高清 (720p)</el-dropdown-item>
                                <el-dropdown-item command="480p">标清 (480p)</el-dropdown-item>
                                <el-dropdown-item command="360p">流畅 (360p)</el-dropdown-item>
                                <el-dropdown-item command="auto">自动</el-dropdown-item>
                            </el-dropdown-menu>
                        </template>
                    </el-dropdown>
                </div>
            </div>
        </div>

        <!-- 视频统计信息（底部独立区域） -->
        <div class="video-stats-bar" v-if="wsStore.isConnected && isReceivingVideo">
            <el-row :gutter="10">
                <el-col :xs="12" :sm="6">
                    <el-statistic title="帧率" :value="videoStats.fps" suffix="FPS" />
                </el-col>
                <el-col :xs="12" :sm="6">
                    <el-statistic title="分辨率" :value="`${videoStats.width}x${videoStats.height}`" />
                </el-col>
                <el-col :xs="12" :sm="6">
                    <el-statistic title="比特率" :value="videoStats.bitrate" suffix="kbps" />
                </el-col>
                <el-col :xs="12" :sm="6">
                    <el-statistic title="延迟" :value="videoStats.latency" suffix="ms" />
                </el-col>
            </el-row>
        </div>
    </div>
</template>

<script setup lang="ts">
import { ref, onMounted, onUnmounted, nextTick, watch } from 'vue'
import { useWebSocketStore } from '../stores/websocket'
import { useAudioStore } from '../stores/audio'
import { ElMessage } from 'element-plus'
import { ArrowDown } from '@element-plus/icons-vue'

const wsStore = useWebSocketStore()
const audioStore = useAudioStore()

// 模板引用
const videoContainer = ref<HTMLDivElement>()
const videoCanvas = ref<HTMLCanvasElement>()

// 视频状态
const isReceivingVideo = ref(false)
const currentQuality = ref('720p')
const canvasWidth = ref(1280)
const canvasHeight = ref(720)

// 视频统计
const videoStats = ref({
    fps: 0,
    width: 1280,
    height: 720,
    bitrate: 0,
    latency: 0
})

// H.264 解码相关
let decoder: any = null
let lastFrameTime = 0
let frameCount = 0
let bitrateSum = 0

onMounted(async () => {
    await initVideoDecoder()
    setupVideoReceiver()
    setupAudioReceiver()
    startFPSCounter()

    // 监听窗口大小变化
    window.addEventListener('resize', handleResize)
    handleResize()
})

onUnmounted(() => {
    cleanup()
    window.removeEventListener('resize', handleResize)
})

// 监听连接状态变化
watch(() => wsStore.isConnected, (connected) => {
    if (connected) {
        setupVideoReceiver()
        setupAudioReceiver()
    } else {
        isReceivingVideo.value = false
    }
})

/**
 * 初始化视频解码器
 */
async function initVideoDecoder() {
    try {
        // 检查浏览器是否支持 WebCodecs
        if ('VideoDecoder' in window) {
            decoder = new (window as any).VideoDecoder({
                output: handleVideoFrame,
                error: (error: any) => {
                    console.error('视频解码错误:', error)
                    ElMessage.error('视频解码失败')
                }
            })

            decoder.configure({
                codec: 'avc1.42E01E', // H.264 Baseline Profile
                optimizeForLatency: true
            })

            console.log('H.264硬件解码器初始化成功')
        } else {
            console.warn('浏览器不支持WebCodecs，使用软件解码')
            // 这里可以实现软件解码的fallback
        }
    } catch (error) {
        console.error('视频解码器初始化失败:', error)
    }
}

/**
 * 设置视频数据接收
 */
function setupVideoReceiver() {
    if (!wsStore.isConnected) return

    // 监听视频数据
    wsStore.onMessage('/robot/video', handleVideoData)
}

/**
 * 处理接收到的视频数据
 */
function handleVideoData(data: any) {
    if (!isReceivingVideo.value) {
        isReceivingVideo.value = true
        ElMessage.success('开始接收视频流')
    }

    try {
        // 将二进制数据转换为 Uint8Array
        let videoData: Uint8Array

        if (data instanceof ArrayBuffer) {
            videoData = new Uint8Array(data)
        } else if (data instanceof Blob) {
            data.arrayBuffer().then(buffer => {
                handleVideoData(buffer)
            })
            return
        } else if (typeof data === 'string') {
            // 如果是base64编码的数据
            const binaryString = atob(data)
            videoData = new Uint8Array(binaryString.length)
            for (let i = 0; i < binaryString.length; i++) {
                videoData[i] = binaryString.charCodeAt(i)
            }
        } else {
            console.warn('未知的视频数据格式:', typeof data)
            return
        }

        // 更新统计信息
        frameCount++
        bitrateSum += videoData.length

        // 如果有硬件解码器，使用硬件解码
        if (decoder && decoder.state === 'configured') {
            decodeH264Frame(videoData)
        } else {
            // 使用软件解码或直接显示（简化实现）
            drawPlaceholderFrame()
        }

    } catch (error) {
        console.error('处理视频数据失败:', error)
    }
}

/**
 * 设置音频数据接收
 */
function setupAudioReceiver() {
    if (!wsStore.isConnected) return

    // 监听音频数据
    wsStore.onMessage('/robot/audio', handleAudioData)
}

/**
 * 处理接收到的音频数据
 */
function handleAudioData(data: any) {
    if (data instanceof Blob) {
        data.arrayBuffer().then(buffer => {
            audioStore.playAudioData(buffer)
        })
    } else if (data instanceof ArrayBuffer) {
        audioStore.playAudioData(data)
    }
}

/**
 * H.264硬件解码
 */
function decodeH264Frame(data: Uint8Array) {
    try {
        const chunk = new (window as any).EncodedVideoChunk({
            type: 'key', // 简化处理，实际需要解析NALU类型
            timestamp: performance.now() * 1000,
            data: data
        })

        decoder.decode(chunk)
    } catch (error) {
        console.error('H.264解码失败:', error)
        drawPlaceholderFrame()
    }
}

/**
 * 处理解码后的视频帧
 */
function handleVideoFrame(frame: any) {
    const canvas = videoCanvas.value
    if (!canvas) return

    const ctx = canvas.getContext('2d')
    if (!ctx) return

    try {
        // 更新画布尺寸
        if (frame.displayWidth !== videoStats.value.width ||
            frame.displayHeight !== videoStats.value.height) {
            videoStats.value.width = frame.displayWidth
            videoStats.value.height = frame.displayHeight
            updateCanvasSize()
        }

        // 绘制视频帧
        ctx.drawImage(frame, 0, 0, canvas.width, canvas.height)

        // 计算延迟
        const now = performance.now()
        if (lastFrameTime > 0) {
            videoStats.value.latency = Math.round(now - lastFrameTime)
        }
        lastFrameTime = now

        frame.close()
    } catch (error) {
        console.error('渲染视频帧失败:', error)
    }
}

/**
 * 绘制占位帧（无视频时显示）
 */
function drawPlaceholderFrame() {
    const canvas = videoCanvas.value
    if (!canvas) return

    const ctx = canvas.getContext('2d')
    if (!ctx) return

    ctx.fillStyle = '#000000'
    ctx.fillRect(0, 0, canvas.width, canvas.height)

    ctx.fillStyle = '#ffffff'
    ctx.font = '24px Arial'
    ctx.textAlign = 'center'
    ctx.fillText('视频流处理中...', canvas.width / 2, canvas.height / 2)
}

/**
 * 启动FPS计数器
 */
function startFPSCounter() {
    setInterval(() => {
        videoStats.value.fps = frameCount
        videoStats.value.bitrate = Math.round(bitrateSum * 8 / 1024) // 转换为kbps

        frameCount = 0
        bitrateSum = 0
    }, 1000)
}

/**
 * 处理画质变化
 */
function handleQualityChange(quality: string) {
    currentQuality.value = quality
    ElMessage.info(`画质设置为: ${quality}`)

    // 这里可以发送画质设置命令到后端
    // wsStore.sendMessage('/video/quality', { quality })
}

/**
 * 处理窗口大小变化
 */
function handleResize() {
    nextTick(() => {
        updateCanvasSize()
    })
}

/**
 * 更新画布尺寸
 */
function updateCanvasSize() {
    const container = videoContainer.value
    if (!container) return

    const containerWidth = container.clientWidth
    const containerHeight = container.clientHeight - 80 // 减去控制栏高度

    const videoAspectRatio = videoStats.value.width / videoStats.value.height
    const containerAspectRatio = containerWidth / containerHeight

    if (containerAspectRatio > videoAspectRatio) {
        // 容器更宽，以高度为准
        canvasHeight.value = containerHeight
        canvasWidth.value = containerHeight * videoAspectRatio
    } else {
        // 容器更高，以宽度为准
        canvasWidth.value = containerWidth
        canvasHeight.value = containerWidth / videoAspectRatio
    }
}

/**
 * 清理资源
 */
function cleanup() {
    if (decoder && decoder.state !== 'closed') {
        decoder.close()
    }

    // 移除消息监听
    wsStore.offMessage('/robot/video', handleVideoData)
    wsStore.offMessage('/robot/audio', handleAudioData)
}
</script>

<style scoped>
.video-player {
    /* 从 App.vue 迁移的布局样式 */
    height: 500px;  /* 固定高度，确保系统信息有空间显示 */
    border-radius: 8px;
    overflow: visible;  /* 允许画质按钮显示在外面 */
    position: relative;  /* 建立定位上下文 */
    z-index: 1;  /* 确保不遮挡下方SystemInfo */

    /* 原有样式 */
    display: flex;
    flex-direction: column;
}

.video-container {
    flex: 1;
    position: relative;
    background-color: #000;
    display: flex;
    justify-content: center;
    align-items: center;
    overflow: hidden;  /* 限制视频内容，防止溢出遮挡 */
    border-radius: 8px;  /* 保持圆角 */
}

.video-canvas {
    max-width: 100%;
    max-height: 100%;
    object-fit: contain;
}

.video-overlay {
    position: absolute;
    top: 0;
    left: 0;
    right: 0;
    bottom: 0;
    pointer-events: none;
    z-index: 10;
}

.video-status {
    position: absolute;
    top: 20px;
    left: 20px;
    pointer-events: auto;
}

/* 画质按钮（浮动在视频右上角） */
.video-quality-btn {
    position: absolute;
    top: 20px;
    right: 20px;
    pointer-events: auto;
    z-index: 20;
}

.video-quality-btn :deep(.el-button) {
    background-color: rgba(0, 0, 0, 0.6);
    border-color: rgba(255, 255, 255, 0.2);
    backdrop-filter: blur(10px);
}

.video-quality-btn :deep(.el-button):hover {
    background-color: rgba(0, 0, 0, 0.8);
    border-color: rgba(255, 255, 255, 0.4);
}

/* 视频统计信息栏（底部独立区域） */
.video-stats-bar {
    padding: 15px;
    background: linear-gradient(to bottom, #f8f9fa 0%, #e9ecef 100%);
    border-top: 1px solid #dee2e6;
}

/* 移动端优化 */
@media screen and (max-width: 768px) {
    /* 视频播放器布局 */
    .video-player {
        height: 300px;  /* 移动端固定高度 */
        border-radius: 4px;
    }

    /* 视频状态标签 - 改到底部左侧，避免与刘海/状态栏重叠 */
    .video-status {
        top: auto;
        bottom: 20px;
        left: 20px;
        transform: none;
    }

    /* 画质按钮 - 移动端保持在右上角 */
    .video-quality-btn {
        top: 15px;
        right: 15px;
    }

    .video-quality-btn :deep(.el-button) {
        min-height: 36px;
        font-size: 13px;
    }

    /* 统计信息栏 - 移动端紧凑布局 */
    .video-stats-bar {
        padding: 12px;
    }

    .video-stats-bar :deep(.el-col) {
        margin-bottom: 10px;
    }
}

/* 小屏手机优化 */
@media screen and (max-width: 480px) {
    .video-player {
        height: 250px;  /* 小屏手机固定高度 */
    }

    .video-status {
        bottom: 15px;
        left: 15px;
    }

    .video-status :deep(.el-tag) {
        font-size: 12px;
        padding: 4px 8px;
    }

    .video-quality-btn {
        top: 12px;
        right: 12px;
    }

    .video-quality-btn :deep(.el-button) {
        font-size: 12px;
        padding: 8px 12px;
        min-height: 32px;
    }

    .video-stats-bar {
        padding: 10px;
    }
}

/* 大屏适配 (1920px 以上) */
@media screen and (min-width: 1920px) {
    .video-player {
        height: 700px;  /* 大屏固定高度 */
    }
}

/* 横屏模式优化 */
@media screen and (max-width: 768px) and (orientation: landscape) {
    .video-player {
        height: 250px;  /* 横屏模式固定高度 */
    }

    .video-status {
        top: 15px;
        bottom: auto;
        left: 15px;
    }

    .video-quality-btn {
        top: 15px;
        right: 15px;
    }
}
</style>
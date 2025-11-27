<template>
    <div class="control-panel">
        <!-- 机器人控制 -->
        <el-card header="机器人控制" class="control-card">
            <div class="control-section">
                <h4>运动控制</h4>

                <!-- 双摇杆控制器 -->
                <div class="joystick-controls">
                    <!-- 左摇杆：移动控制 -->
                    <div class="joystick-wrapper">
                        <div class="joystick-label">移动控制</div>
                        <div id="left-joystick" class="joystick-container"></div>
                        <div class="joystick-hint">前后 + 左右</div>
                    </div>

                    <!-- 右摇杆：旋转控制 -->
                    <div class="joystick-wrapper">
                        <div class="joystick-label">旋转控制</div>
                        <div id="right-joystick" class="joystick-container"></div>
                        <div class="joystick-hint">左转 / 右转</div>
                    </div>
                </div>

                <!-- 姿态控制 -->
                <div class="state-controls">
                    <el-button @click="setRunState('lie')" :disabled="!wsStore.isConnected" class="control-btn">
                        趴下
                    </el-button>
                    <el-button @click="setRunState('stand')" :disabled="!wsStore.isConnected" class="control-btn">
                        站立
                    </el-button>
                    <el-button @click="setRunState('walk')" :disabled="!wsStore.isConnected" class="control-btn">
                        行走
                    </el-button>
                    <el-button @click="stopRobot" :disabled="!wsStore.isConnected" class="control-btn stop-btn">
                        停止
                    </el-button>
                </div>

                <!-- 云台控制 -->
                <div class="ptz-controls">
                    <h4>云台控制</h4>
                    <div class="ptz-grid">
                        <div></div>
                        <el-button @click="controlCameraPtz(0.5, 0)" :disabled="!wsStore.isConnected" class="ptz-btn">
                            ↑
                        </el-button>
                        <div></div>
                        <el-button @click="controlCameraPtz(0, -0.5)" :disabled="!wsStore.isConnected" class="ptz-btn">
                            ←
                        </el-button>
                        <el-button @click="controlCameraPtz(0, 0)" :disabled="!wsStore.isConnected" class="ptz-btn">
                            ●
                        </el-button>
                        <el-button @click="controlCameraPtz(0, 0.5)" :disabled="!wsStore.isConnected" class="ptz-btn">
                            →
                        </el-button>
                        <div></div>
                        <el-button @click="controlCameraPtz(-0.5, 0)" :disabled="!wsStore.isConnected" class="ptz-btn">
                            ↓
                        </el-button>
                        <div></div>
                    </div>
                </div>
            </div>
        </el-card>

        <!-- 音频控制 -->
        <el-card header="音频控制" class="control-card">
            <div class="control-section">
                <div class="audio-controls">
                    <el-button :type="audioStore.isMuted ? 'danger' : 'success'" @click="toggleAudio"
                        :disabled="!wsStore.isConnected" style="width: 45%">
                        {{ audioStore.isMuted ? '开音' : '静音' }}
                    </el-button>

                    <el-button :type="audioStore.isTalking ? 'danger' : 'primary'" @click="toggleTalk"
                        :disabled="!wsStore.isConnected" style="width: 45%">
                        {{ audioStore.isTalking ? '关闭对讲' : '开启对讲' }}
                    </el-button>
                </div>

                <div class="volume-control">
                    <el-text>音量控制</el-text>
                    <el-slider v-model="audioStore.volume" :min="0" :max="100" @change="onVolumeChange"
                        :disabled="!wsStore.isConnected || audioStore.isMuted" />
                </div>
            </div>
        </el-card>
    </div>
</template>

<script setup lang="ts">
import { useWebSocketStore } from '../stores/websocket'
import { useAudioStore } from '../stores/audio'
import { useJoystick } from '../composables/useJoystick'
import { ElMessage } from 'element-plus'

const wsStore = useWebSocketStore()
const audioStore = useAudioStore()

// 速度命令回调函数
function handleVelocityChange(vx: number, vy: number, az: number) {
    if (!wsStore.isConnected) return

    const success = wsStore.sendRobotCommand('/quadruped/set_velocity', {
        linear: [`${vx.toFixed(2)}`, `${vy.toFixed(2)}`, "0"],
        angular: ["0", "0", `${az.toFixed(2)}`]
    })

    if (!success) {
        console.error('发送速度命令失败')
    }
}

// 初始化双摇杆（直接在顶层调用，不要放在 onMounted 里）
useJoystick(
    {
        containerId: 'left-joystick',
        color: '#3b82f6',
        size: 140  // 修改：与 CSS 容器尺寸一致，修复 iOS Safari 位置偏移
    },
    {
        containerId: 'right-joystick',
        color: '#3b82f6',
        size: 140  // 修改：与 CSS 容器尺寸一致，修复 iOS Safari 位置偏移
        // 不设置 lock 参数，让 nipplejs 自由移动，只在代码中使用 X 轴数据
    },
    handleVelocityChange
)

// 设置机器人运行状态
function setRunState(state: 'lie' | 'stand' | 'walk') {
    const success = wsStore.sendRobotCommand('/quadruped/set_run_state', {
        run_state: state
    })

    if (success) {
        ElMessage.success(`设置状态: ${state}`)
    } else {
        ElMessage.error('发送状态命令失败')
    }
}

// 停止机器人
function stopRobot() {
    const success = wsStore.sendRobotCommand('/quadruped/set_velocity', {
        linear: ["0", "0", "0"],
        angular: ["0", "0", "0"]
    })

    if (success) {
        ElMessage.info('已停止')
    } else {
        ElMessage.error('发送停止命令失败')
    }
}

// 控制摄像头云台
function controlCameraPtz(vx: number, vy: number) {
    const success = wsStore.sendRobotCommand('/device/set_camera_ptz', {
        vx: vx,
        vy: vy
    })

    if (!success) {
        ElMessage.error('云台控制失败')
    }
}

// 音频控制
function toggleAudio() {
    audioStore.toggleMute()
}

function toggleTalk() {
    audioStore.toggleTalk()
}

function onVolumeChange(value: number) {
    audioStore.setVolume(value)
}
</script>

<style scoped>
/* ===== 卡片样式 ===== */
.control-card {
    background-color: #2c3e50;
    border: 1px solid #34495e;
    border-radius: 8px;
    margin-bottom: 16px;
    box-shadow: 0 2px 8px rgba(0, 0, 0, 0.3);
}

.control-card :deep(.el-card__header) {
    background-color: #34495e;
    color: #ecf0f1;
    border-bottom: 2px solid #3498db;
    font-weight: 600;
    padding: 12px 16px;
}

.control-card :deep(.el-card__body) {
    padding: 16px;
}

/* ===== 控制区块 ===== */
.control-section {
    display: flex;
    flex-direction: column;
    gap: 16px;
}

.control-section h4 {
    margin: 0 0 12px 0;
    color: #ecf0f1;
    font-size: 14px;
    font-weight: 600;
    text-transform: uppercase;
    letter-spacing: 0.5px;
}

/* ===== 双摇杆控制器 ===== */
.joystick-controls {
    display: flex;
    flex-direction: column;
    gap: 20px;
    padding: 16px;
    background: linear-gradient(145deg, #1a1d23 0%, #0f1115 100%);
    border-radius: 12px;
    box-shadow:
        0 8px 24px rgba(0, 0, 0, 0.6),
        inset 0 1px 0 rgba(255, 255, 255, 0.05),
        inset 0 -2px 8px rgba(0, 0, 0, 0.4);
}

.joystick-wrapper {
    display: flex;
    flex-direction: column;
    align-items: center;
    gap: 8px;
}

.joystick-label {
    font-size: 13px;
    font-weight: 600;
    color: #9ca3af;
    text-transform: uppercase;
    letter-spacing: 0.5px;
}

.joystick-container {
    width: 140px;
    height: 140px;
    position: relative;
    border-radius: 50%;
    background: radial-gradient(circle, #374151 0%, #1f2937 100%);
    box-shadow:
        inset 0 2px 8px rgba(0, 0, 0, 0.5),
        0 4px 12px rgba(0, 0, 0, 0.4);

    /* iOS Safari 优化：修复摇杆球位置偏移问题 */
    -webkit-transform: translateZ(0);  /* 启用硬件加速 */
    transform: translateZ(0);
    contain: layout size style;  /* 隔离布局计算 */
    flex-shrink: 0;  /* 防止 flex 容器压缩 */
    flex-grow: 0;    /* 防止 flex 容器扩展 */

    /* 确保触摸事件正确 */
    touch-action: none;  /* 禁用浏览器默认触摸行为 */
    -webkit-user-select: none;
    user-select: none;
}

/* ===== nipplejs 摇杆样式（增强版） ===== */

/* 确保所有 nipplejs 子元素可见 */
.joystick-container :deep(*) {
    pointer-events: auto !important;
}

/* SVG 元素（如果使用 SVG 渲染） */
.joystick-container :deep(svg) {
    width: 100%;
    height: 100%;
    position: absolute;
    top: 0;
    left: 0;
    z-index: 10;
}

/* 后端圆圈（背景 - 外圈）- 隐藏白色圆圈 */
.joystick-container :deep(.back) {
    /* 针对 div 元素 - 使用透明背景，去掉边框 */
    background: transparent !important;
    border: none !important;
    border-radius: 50% !important;
    opacity: 0 !important;  /* 完全隐藏外圈 */

    /* 针对 SVG 元素 */
    fill: transparent !important;
    stroke: none !important;
    stroke-width: 0 !important;
}

/* 前端圆圈（摇杆点 - 可拖动的内部圆点）- 超强可见度 */
.joystick-container :deep(.front) {
    /* 针对 div 元素 */
    background: rgba(59, 130, 246, 0.95) !important;
    border: 3px solid rgba(96, 165, 250, 1) !important;
    border-radius: 50% !important;
    box-shadow:
        0 0 12px rgba(59, 130, 246, 0.8),
        0 0 24px rgba(59, 130, 246, 0.5),
        inset 0 2px 4px rgba(255, 255, 255, 0.3) !important;
    opacity: 1 !important;
    display: block !important;
    visibility: visible !important;

    /* 针对 SVG 元素 */
    fill: rgba(59, 130, 246, 0.95) !important;
    stroke: rgba(96, 165, 250, 1) !important;
    stroke-width: 3px !important;
    filter: drop-shadow(0 0 12px rgba(59, 130, 246, 0.8));
}

/* 确保所有圆形元素可见 */
.joystick-container :deep(circle) {
    opacity: 1 !important;
    display: block !important;
}

/* 针对 nipplejs 动态生成的所有子元素 */
.joystick-container > * {
    opacity: 1 !important;
    visibility: visible !important;
}

.joystick-hint {
    font-size: 11px;
    color: #6b7280;
    margin-top: 4px;
}

/* ===== 姿态控制按钮 ===== */
.state-controls {
    display: grid;
    grid-template-columns: repeat(2, 1fr);  /* 恢复占满屏幕 */
    gap: 10px;
    margin-top: 12px;
    padding: 0;  /* 确保容器无内边距 */
}

.control-btn {
    width: 100% !important;  /* 强制所有按钮宽度一致 */
    height: 40px !important;
    min-width: unset !important;  /* 移除 Element Plus 默认最小宽度 */
    margin: 0 !important;  /* 强制移除所有外边距 */
    padding: 0 !important;  /* 统一内边距 */
    font-size: 14px !important;  /* 统一字体大小 */
    font-weight: 600;
    background: linear-gradient(145deg, #34495e 0%, #2c3e50 100%);
    border: 1px solid #3498db;
    color: #ecf0f1;
    transition: all 0.2s ease;
    display: flex !important;
    align-items: center !important;
    justify-content: center !important;
    box-sizing: border-box !important;  /* 确保边框计入宽度 */
}

.control-btn:hover:not(:disabled) {
    background: linear-gradient(145deg, #3498db 0%, #2980b9 100%);
    transform: translateY(-1px);
    box-shadow: 0 4px 12px rgba(52, 152, 219, 0.4);
}

.control-btn:active:not(:disabled) {
    transform: translateY(0);
    box-shadow: 0 2px 6px rgba(52, 152, 219, 0.3);
}

.stop-btn {
    border-color: #e74c3c;
}

.stop-btn:hover:not(:disabled) {
    background: linear-gradient(145deg, #e74c3c 0%, #c0392b 100%);
    border-color: #e74c3c;
    box-shadow: 0 4px 12px rgba(231, 76, 60, 0.4);
}

/* ===== 云台控制 ===== */
.ptz-controls {
    margin-top: 8px;
}

.ptz-grid {
    display: grid;
    grid-template-columns: repeat(3, 48px);  /* 固定每列宽度 */
    grid-template-rows: repeat(3, 48px);     /* 固定每行高度 */
    gap: 8px;
    justify-content: center;  /* 水平居中 */
    margin: 0 auto;
}

.ptz-btn {
    height: 48px;
    width: 48px;
    padding: 0;
    margin: 0;
    font-size: 18px;
    font-weight: 700;
    line-height: 48px;      /* 与高度一致，确保垂直居中 */
    background: linear-gradient(145deg, #34495e 0%, #2c3e50 100%);
    border: 1px solid #3498db;
    color: #ecf0f1;
    transition: all 0.2s ease;
    display: flex;          /* 使用 flexbox 居中内容 */
    align-items: center;    /* 垂直居中 */
    justify-content: center; /* 水平居中 */
}

/* 确保按钮内的文字完全居中 */
.ptz-btn :deep(span) {
    display: flex;
    align-items: center;
    justify-content: center;
    width: 100%;
    height: 100%;
}

.ptz-btn:hover:not(:disabled) {
    background: linear-gradient(145deg, #3498db 0%, #2980b9 100%);
    transform: scale(1.05);
    box-shadow: 0 4px 12px rgba(52, 152, 219, 0.5);
}

.ptz-btn:active:not(:disabled) {
    transform: scale(0.95);
}

/* ===== 音频控制 ===== */
.audio-controls {
    display: flex;
    justify-content: space-between;
    gap: 10px;
}

.volume-control {
    margin-top: 12px;
}

.volume-control .el-text {
    color: #ecf0f1;
    font-size: 13px;
    margin-bottom: 8px;
    display: block;
}

/* ===== 禁用状态 ===== */
.control-btn:disabled,
.ptz-btn:disabled {
    opacity: 0.4;
    cursor: not-allowed;
    background: #2c3e50;
    border-color: #34495e;
}

/* ===== 响应式适配 ===== */
@media screen and (max-width: 768px) {
    .joystick-controls {
        padding: 12px;
        gap: 16px;
    }

    .joystick-container {
        width: 120px;
        height: 120px;
    }

    .control-btn,
    .ptz-btn {
        height: 40px;
    }

    .ptz-grid {
        max-width: 140px;
    }
}

@media screen and (max-width: 480px) {
    .joystick-controls {
        padding: 8px;
        gap: 12px;
    }

    .joystick-container {
        width: 100px;
        height: 100px;
    }

    .control-btn {
        height: 36px;
        font-size: 13px;
    }

    .ptz-btn {
        height: 38px;
        width: 38px;
        font-size: 14px;
    }

    .ptz-grid {
        max-width: 120px;
    }
}
</style>

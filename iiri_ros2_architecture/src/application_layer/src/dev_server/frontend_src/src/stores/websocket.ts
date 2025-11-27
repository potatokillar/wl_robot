import { defineStore } from 'pinia'
import { ref, reactive } from 'vue'
import { fetchDeviceUUID } from '@/api/robot'

export interface ConnectionConfig {
    ip: string
    uuid: string
    port: number
}

export interface WebSocketMessage {
    type: string
    sequence?: string
    param?: any
    timestamp?: number
}

// 频道元数据
export interface ChannelMetadata {
    displayName: string    // 中文显示名称
    category: string       // 分类（媒体流/控制命令）
    icon: string          // Element Plus 图标名
}

// 频道连接信息
export interface ChannelConnection {
    ws: WebSocket
    displayName: string
    category: string
    icon: string
    connectedAt: Date
}

// 频道元数据字典
const CHANNEL_METADATA: Record<string, ChannelMetadata> = {
    '/robot/video': { displayName: '机器人视频', category: '媒体流', icon: 'VideoCamera' },
    '/robot/audio': { displayName: '机器人音频', category: '媒体流', icon: 'Microphone' },
    '/user/audio': { displayName: '用户音频', category: '媒体流', icon: 'Headset' },
    '/device/set_camera_ptz': { displayName: '云台控制', category: '控制命令', icon: 'Camera' },
    '/quadruped/set_velocity': { displayName: '速度控制', category: '控制命令', icon: 'Aim' },
    '/quadruped/set_run_state': { displayName: '状态控制', category: '控制命令', icon: 'Setting' },
    '/device/set_talk': { displayName: '对讲控制', category: '控制命令', icon: 'ChatLineSquare' }
}

export const useWebSocketStore = defineStore('websocket', () => {
    // 连接状态
    const isConnected = ref(false)
    const isConnecting = ref(false)
    const connectionError = ref<string>('')

    // 连接配置（自动获取主机名和UUID）
    const config = reactive<ConnectionConfig>({
        ip: window.location.hostname || '192.168.1.54',
        uuid: '',  // 初始为空，将通过 initUUID() 自动获取
        port: 20555  // dev_server WebSocket 端口
    })

    // WebSocket连接（存储连接详情）
    const connections = new Map<string, ChannelConnection>()
    const messageCallbacks = new Map<string, ((data: any) => void)[]>()

    /**
     * 初始化 UUID（从 dev_server 获取）
     */
    async function initUUID() {
        try {
            config.uuid = await fetchDeviceUUID()
            console.log('Device UUID obtained:', config.uuid)
            return true
        } catch (error) {
            console.error('Failed to fetch device UUID:', error)
            connectionError.value = 'Failed to fetch device UUID'
            return false
        }
    }

    /**
     * 连接到WebSocket服务器
     */
    async function connect() {
        if (isConnecting.value) return

        try {
            isConnecting.value = true
            connectionError.value = ''

            // 连接主要频道
            await connectChannel('/robot/video')
            await connectChannel('/robot/audio')
            await connectChannel('/user/audio')
            await connectChannel('/device/set_camera_ptz')
            await connectChannel('/quadruped/set_velocity')
            await connectChannel('/quadruped/set_run_state')
            await connectChannel('/device/set_talk')

            isConnected.value = true
            console.log('WebSocket连接成功')
        } catch (error) {
            console.error('WebSocket连接失败:', error)
            connectionError.value = error instanceof Error ? error.message : '连接失败'
            isConnected.value = false
        } finally {
            isConnecting.value = false
        }
    }

    /**
     * 连接指定频道
     */
    function connectChannel(channel: string): Promise<void> {
        return new Promise((resolve, reject) => {
            if (!channel.startsWith('/')) {
                reject(new Error('频道路径必须以/开头'))
                return
            }

            // dev_server 格式: ws://{ip}:20555/{uuid}/{role}/{channel}
            // 例如: ws://192.168.1.54:20555/6f06e9cb0071/usr/robot/video
            // role 必须是 3 个字符: "usr" 表示用户端, "dev" 表示设备端
            const cleanChannel = channel.startsWith('/') ? channel.slice(1) : channel
            const url = `ws://${config.ip}:${config.port}/${config.uuid}/usr/${cleanChannel}`
            console.log('正在连接WebSocket频道:', url)

            const ws = new WebSocket(url)

            ws.onopen = () => {
                console.log(`WebSocket频道连接成功: ${channel}`)

                // 获取频道元数据（如果没有则使用默认值）
                const metadata = CHANNEL_METADATA[channel] || {
                    displayName: channel,
                    category: '未分类',
                    icon: 'Connection'
                }

                // 存储连接详情
                connections.set(channel, {
                    ws,
                    displayName: metadata.displayName,
                    category: metadata.category,
                    icon: metadata.icon,
                    connectedAt: new Date()
                })
                resolve()
            }

            ws.onmessage = (event) => {
                handleMessage(channel, event.data)
            }

            ws.onclose = (event) => {
                console.log(`WebSocket频道关闭: ${channel} - ${event.code} - ${event.reason}`)
                connections.delete(channel)

                // 只要有一个连接关闭，就认为连接断开
                isConnected.value = false
            }

            ws.onerror = (error) => {
                console.error(`WebSocket频道连接错误: ${channel}`, error)
                connections.delete(channel)
                // 发生错误时，也认为连接断开
                isConnected.value = false
                reject(error)
            }
        })
    }

    /**
     * 断开所有连接
     */
    function disconnect() {
        // 先设置状态，防止后续回调干扰
        isConnecting.value = false
        isConnected.value = false

        connections.forEach((conn, channel) => {
            console.log(`关闭WebSocket频道: ${channel}`)
            // 清除事件监听器，防止关闭回调干扰
            conn.ws.onclose = null
            conn.ws.onerror = null
            conn.ws.close(1000, '正常关闭')
        })
        connections.clear()
        messageCallbacks.clear()
    }

    /**
     * 发送文本消息到指定频道
     */
    function sendMessage(channel: string, message: string | object) {
        const conn = connections.get(channel)
        if (!conn || conn.ws.readyState !== WebSocket.OPEN) {
            console.error(`频道 ${channel} 未连接或连接已关闭`)
            return false
        }

        try {
            const data = typeof message === 'string' ? message : JSON.stringify(message)
            conn.ws.send(data)
            console.log(`发送消息到 ${channel}:`, data)
            return true
        } catch (error) {
            console.error(`发送消息失败:`, error)
            return false
        }
    }

    /**
     * 发送二进制数据到指定频道
     */
    function sendBinary(channel: string, data: ArrayBuffer | Blob) {
        const conn = connections.get(channel)
        if (!conn || conn.ws.readyState !== WebSocket.OPEN) {
            console.error(`频道 ${channel} 未连接或连接已关闭`)
            return false
        }

        try {
            conn.ws.send(data)
            console.log(`发送二进制数据到 ${channel}:`, data.constructor.name, data)
            return true
        } catch (error) {
            console.error(`发送二进制数据失败:`, error)
            return false
        }
    }

    /**
     * 注册消息回调
     */
    function onMessage(channel: string, callback: (data: any) => void) {
        if (!messageCallbacks.has(channel)) {
            messageCallbacks.set(channel, [])
        }
        messageCallbacks.get(channel)!.push(callback)
    }

    /**
     * 移除消息回调
     */
    function offMessage(channel: string, callback: (data: any) => void) {
        const callbacks = messageCallbacks.get(channel)
        if (callbacks) {
            const index = callbacks.indexOf(callback)
            if (index > -1) {
                callbacks.splice(index, 1)
            }
        }
    }

    /**
     * 处理收到的消息
     */
    function handleMessage(channel: string, data: any) {
        console.log(`收到消息 ${channel}:`, data)

        // 调用注册的回调函数
        const callbacks = messageCallbacks.get(channel)
        if (callbacks) {
            callbacks.forEach(callback => {
                try {
                    callback(data)
                } catch (error) {
                    console.error('消息回调执行错误:', error)
                }
            })
        }
    }

    /**
     * 发送机器人控制命令
     */
    function sendRobotCommand(command: string, params: any) {
        const message: WebSocketMessage = {
            type: 'pub',
            sequence: generateSequence(),
            param: params,
            timestamp: Date.now()
        }

        return sendMessage(command, message)
    }

    /**
     * 生成随机序列号
     */
    function generateSequence(): string {
        return Math.random().toString(36).substring(2, 10)
    }

    return {
        // 状态
        isConnected,
        isConnecting,
        connectionError,
        config,

        // 方法
        initUUID,  // 新增：初始化 UUID
        connect,
        disconnect,
        connectChannel,
        sendMessage,
        sendBinary,
        sendRobotCommand,
        onMessage,
        offMessage,

        // 计算属性
        connectionCount: () => {
            let activeCount = 0
            connections.forEach((conn) => {
                if (conn.ws.readyState === WebSocket.OPEN) {
                    activeCount++
                }
            })
            return activeCount
        },
        getConnection: (channel: string) => connections.get(channel),

        /**
         * 获取所有连接的详细信息（供UI显示）
         */
        getConnectionDetails: () => {
            const details: Array<{
                channel: string
                displayName: string
                category: string
                icon: string
                status: 'connected' | 'connecting' | 'disconnected'
                connectedAt?: Date
            }> = []

            connections.forEach((conn, channel) => {
                let status: 'connected' | 'connecting' | 'disconnected' = 'disconnected'
                if (conn.ws.readyState === WebSocket.OPEN) {
                    status = 'connected'
                } else if (conn.ws.readyState === WebSocket.CONNECTING) {
                    status = 'connecting'
                }

                details.push({
                    channel,
                    displayName: conn.displayName,
                    category: conn.category,
                    icon: conn.icon,
                    status,
                    connectedAt: status === 'connected' ? conn.connectedAt : undefined
                })
            })

            return details
        }
    }
})
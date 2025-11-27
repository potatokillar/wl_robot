import { defineStore } from 'pinia'
import { reactive } from 'vue'

export interface RobotStatus {
    runState: 'stand' | 'lie' | 'walk' | 'unknown'
    battery: number
    position: {
        x: number
        y: number
        z: number
    }
    orientation: {
        roll: number
        pitch: number
        yaw: number
    }
    velocity: {
        linear: [number, number, number]
        angular: [number, number, number]
    }
    connected: boolean
    lastUpdate: number
}

export const useRobotStore = defineStore('robot', () => {
    // 机器人状态
    const status = reactive<RobotStatus>({
        runState: 'unknown',
        battery: 100,
        position: { x: 0, y: 0, z: 0 },
        orientation: { roll: 0, pitch: 0, yaw: 0 },
        velocity: {
            linear: [0, 0, 0],
            angular: [0, 0, 0]
        },
        connected: false,
        lastUpdate: Date.now()
    })

    /**
     * 更新机器人状态
     */
    function updateStatus(newStatus: Partial<RobotStatus>) {
        Object.assign(status, newStatus)
        status.lastUpdate = Date.now()
    }

    /**
     * 设置连接状态
     */
    function setConnected(connected: boolean) {
        status.connected = connected
        if (!connected) {
            // 断连时重置状态
            status.runState = 'unknown'
            status.velocity.linear = [0, 0, 0]
            status.velocity.angular = [0, 0, 0]
        }
    }

    /**
     * 设置运行状态
     */
    function setRunState(runState: RobotStatus['runState']) {
        status.runState = runState
        status.lastUpdate = Date.now()
    }

    /**
     * 设置电池电量
     */
    function setBattery(battery: number) {
        status.battery = Math.max(0, Math.min(100, battery))
        status.lastUpdate = Date.now()
    }

    /**
     * 设置位置
     */
    function setPosition(x: number, y: number, z: number = 0) {
        status.position.x = x
        status.position.y = y
        status.position.z = z
        status.lastUpdate = Date.now()
    }

    /**
     * 设置姿态
     */
    function setOrientation(roll: number, pitch: number, yaw: number) {
        status.orientation.roll = roll
        status.orientation.pitch = pitch
        status.orientation.yaw = yaw
        status.lastUpdate = Date.now()
    }

    /**
     * 设置速度
     */
    function setVelocity(linear: [number, number, number], angular: [number, number, number]) {
        status.velocity.linear = linear
        status.velocity.angular = angular
        status.lastUpdate = Date.now()
    }

    /**
     * 检查连接状态
     */
    function isStatusFresh(): boolean {
        const now = Date.now()
        return (now - status.lastUpdate) < 5000 // 5秒内的数据认为是新鲜的
    }

    /**
     * 获取机器人健康状态
     */
    function getHealthStatus(): 'good' | 'warning' | 'critical' {
        if (!status.connected || !isStatusFresh()) {
            return 'critical'
        }

        if (status.battery < 20) {
            return 'critical'
        }

        if (status.battery < 50) {
            return 'warning'
        }

        return 'good'
    }

    return {
        // 状态
        status,

        // 方法
        updateStatus,
        setConnected,
        setRunState,
        setBattery,
        setPosition,
        setOrientation,
        setVelocity,
        isStatusFresh,
        getHealthStatus
    }
})
/**
 * 双摇杆控制器钩子
 * 使用 nipplejs 实现双摇杆控制：
 * - 左摇杆：控制移动（linear_x, linear_y）
 * - 右摇杆：控制旋转（angular_z）
 */
import { ref, onMounted, onUnmounted, Ref } from 'vue'
import nipplejs from 'nipplejs'
import type { JoystickManager, EventData } from 'nipplejs'

// 死区阈值：15% 的摇杆半径
// 用于过滤手指轻触产生的微小移动，防止误触发
const DEADZONE_THRESHOLD = 0.15

export interface JoystickConfig {
  containerId: string
  color?: string
  size?: number
  lockX?: boolean
  lockY?: boolean
}

export interface JoystickData {
  x: number  // 归一化值 -1.0 到 1.0
  y: number  // 归一化值 -1.0 到 1.0
  distance: number  // 距离 0.0 到 1.0
  angle: number  // 角度（度）
}

export function useJoystick(
  leftConfig: JoystickConfig,
  rightConfig: JoystickConfig,
  onVelocityChange: (vx: number, vy: number, az: number) => void
) {
  const leftManager = ref<JoystickManager | null>(null)
  const rightManager = ref<JoystickManager | null>(null)

  const leftData = ref<JoystickData>({
    x: 0,
    y: 0,
    distance: 0,
    angle: 0
  })

  const rightData = ref<JoystickData>({
    x: 0,
    y: 0,
    distance: 0,
    angle: 0
  })

  // 当前的速度命令
  let currentVx = 0
  let currentVy = 0
  let currentAz = 0

  // 定时器用于持续发送速度命令
  let sendInterval: number | null = null
  const SEND_INTERVAL_MS = 5  // 每 5ms 发送一次 (200Hz)

  // 启动定时器，持续发送速度命令
  function startSendingVelocity() {
    if (sendInterval !== null) return  // 已经在运行

    console.log('[JOYSTICK] 启动定时器，开始持续发送速度命令')
    sendInterval = window.setInterval(() => {
      // 只要有非零速度，就持续发送
      if (currentVx !== 0 || currentVy !== 0 || currentAz !== 0) {
        console.log('[JOYSTICK] 定时器发送:', { vx: currentVx, vy: currentVy, az: currentAz })
        onVelocityChange(currentVx, currentVy, currentAz)
      }
    }, SEND_INTERVAL_MS)
  }

  // 停止定时器
  function stopSendingVelocity() {
    if (sendInterval !== null) {
      console.log('[JOYSTICK] 停止定时器')
      clearInterval(sendInterval)
      sendInterval = null
    }
    // 发送最后一次 0 速度
    currentVx = 0
    currentVy = 0
    currentAz = 0
    onVelocityChange(0, 0, 0)
  }

  // 创建左摇杆（移动控制）
  function createLeftJoystick() {
    const container = document.getElementById(leftConfig.containerId)
    console.log('[JOYSTICK] 左摇杆容器查找:', leftConfig.containerId, container)

    if (!container) {
      console.error(`[JOYSTICK] ❌ 左摇杆容器未找到: ${leftConfig.containerId}`)
      return
    }

    // 获取容器位置信息用于调试
    const rect = container.getBoundingClientRect()
    console.log('[JOYSTICK] 左摇杆容器位置:', {
      width: rect.width,
      height: rect.height,
      left: rect.left,
      top: rect.top
    })

    console.log('[JOYSTICK] 左摇杆配置:', {
      color: leftConfig.color || '#3b82f6',
      size: leftConfig.size || 140,
      mode: 'static',
      position: { left: '50%', top: '50%' },
      dynamicPage: true
    })

    // 使用 static 模式，明确设置 position 和 dynamicPage（Vue 3 必需）
    leftManager.value = nipplejs.create({
      zone: container,
      mode: 'static' as const,
      position: { left: '50%', top: '50%' },  // 明确设置中心位置
      dynamicPage: true,                      // Vue 3 应用必需
      color: leftConfig.color || '#3b82f6',
      size: leftConfig.size || 140,
      dataOnly: false,
      lockX: leftConfig.lockX || false,
      lockY: leftConfig.lockY || false
    })

    console.log('[JOYSTICK] ✅ 左摇杆 Manager 已创建:', leftManager.value)

    // 打印容器内的 DOM 结构
    setTimeout(() => {
      console.log('[JOYSTICK] 左摇杆容器的 DOM 结构:')
      console.log('  - innerHTML:', container.innerHTML.substring(0, 200))
      console.log('  - childNodes 数量:', container.childNodes.length)
      console.log('  - 子元素:', Array.from(container.children).map(el => ({
        tag: el.tagName,
        class: el.className,
        style: el.getAttribute('style')
      })))
    }, 500)

    // 监听开始事件
    leftManager.value.on('start', () => {
      console.log('[JOYSTICK] 左摇杆 START 事件触发')
    })

    // 监听移动事件
    leftManager.value.on('move', (evt: any, data: EventData) => {
      // nipplejs 坐标系：X右正，Y上正
      // 转换为机器人坐标系：vx前正，vy左正
      const rawDistance = data.distance / (leftConfig.size || 120)  // 归一化到 0-1

      // 死区检查：过滤手指轻触产生的微小移动
      if (rawDistance < DEADZONE_THRESHOLD) {
        console.log('[JOYSTICK] 左摇杆在死区内，忽略移动 (distance=' + rawDistance.toFixed(3) + ')')

        // 强制清零数据，防止旧角度污染
        leftData.value = { x: 0, y: 0, distance: 0, angle: 0 }
        currentVx = 0
        currentVy = 0

        // 如果右摇杆也为 0，停止定时器
        if (currentAz === 0) {
          stopSendingVelocity()
        }

        return
      }

      // 将死区外的距离重新映射到 0-1
      // 公式：(rawDistance - deadzone) / (1 - deadzone)
      const distance = (rawDistance - DEADZONE_THRESHOLD) / (1.0 - DEADZONE_THRESHOLD)
      const angleRad = (data.angle.radian - Math.PI / 2)  // 转换角度使0度朝前

      leftData.value = {
        x: Math.cos(angleRad) * distance,  // 前后
        y: Math.sin(angleRad) * distance,  // 左右
        distance: distance,
        angle: data.angle.degree
      }

      // 更新速度值
      currentVx = leftData.value.x  // 前后运动（-1到1）
      currentVy = leftData.value.y  // 左右运动（-1到1）

      // 第一次移动时启动定时器
      if (sendInterval === null) {
        startSendingVelocity()
      }
    })

    // 监听释放事件
    leftManager.value.on('end', () => {
      console.log('[JOYSTICK] 左摇杆 END 事件触发')
      leftData.value = { x: 0, y: 0, distance: 0, angle: 0 }
      currentVx = 0
      currentVy = 0

      // 如果右摇杆也已释放（角速度为0），停止定时器
      if (currentAz === 0) {
        stopSendingVelocity()
      } else {
        // 右摇杆还在使用，继续发送（但 vx=vy=0）
        onVelocityChange(currentVx, currentVy, currentAz)
      }
    })
  }

  // 创建右摇杆（旋转控制）
  function createRightJoystick() {
    const container = document.getElementById(rightConfig.containerId)
    console.log('[JOYSTICK] 右摇杆容器查找:', rightConfig.containerId, container)

    if (!container) {
      console.error(`[JOYSTICK] ❌ 右摇杆容器未找到: ${rightConfig.containerId}`)
      return
    }

    // 获取容器位置信息用于调试
    const rect = container.getBoundingClientRect()
    console.log('[JOYSTICK] 右摇杆容器位置:', {
      width: rect.width,
      height: rect.height,
      left: rect.left,
      top: rect.top
    })

    console.log('[JOYSTICK] 右摇杆配置:', {
      color: rightConfig.color || '#3b82f6',
      size: rightConfig.size || 140,
      mode: 'static',
      position: { left: '50%', top: '50%' },
      dynamicPage: true,
      lockX: true
    })

    // 使用 static 模式，明确设置 position 和 dynamicPage（Vue 3 必需）
    rightManager.value = nipplejs.create({
      zone: container,
      mode: 'static' as const,
      position: { left: '50%', top: '50%' },  // 明确设置中心位置
      dynamicPage: true,                      // Vue 3 应用必需
      color: rightConfig.color || '#3b82f6',
      size: rightConfig.size || 140,
      dataOnly: false,
      lockX: true,  // 锁定到 X 轴方向，只允许左右移动控制旋转
      lockY: false
    })

    console.log('[JOYSTICK] ✅ 右摇杆 Manager 已创建:', rightManager.value)

    // 打印容器内的 DOM 结构
    setTimeout(() => {
      console.log('[JOYSTICK] 右摇杆容器的 DOM 结构:')
      console.log('  - innerHTML:', container.innerHTML.substring(0, 200))
      console.log('  - childNodes 数量:', container.childNodes.length)
      console.log('  - 子元素:', Array.from(container.children).map(el => ({
        tag: el.tagName,
        class: el.className,
        style: el.getAttribute('style')
      })))
    }, 500)

    // 监听开始事件
    rightManager.value.on('start', () => {
      console.log('[JOYSTICK] 右摇杆 START 事件触发')
    })

    // 监听移动事件
    rightManager.value.on('move', (evt: any, data: EventData) => {
      // 只使用X轴数据来控制旋转
      const rawDistance = data.distance / (rightConfig.size || 120)

      // 死区检查：过滤手指轻触产生的微小移动
      if (rawDistance < DEADZONE_THRESHOLD) {
        console.log('[JOYSTICK] 右摇杆在死区内，忽略移动 (distance=' + rawDistance.toFixed(3) + ')')

        // 强制清零数据，防止旧角度污染
        rightData.value = { x: 0, y: 0, distance: 0, angle: 0 }
        currentAz = 0

        // 如果左摇杆也为 0，停止定时器
        if (currentVx === 0 && currentVy === 0) {
          stopSendingVelocity()
        }

        return
      }

      // 将死区外的距离重新映射到 0-1
      const distance = (rawDistance - DEADZONE_THRESHOLD) / (1.0 - DEADZONE_THRESHOLD)
      const angleRad = data.angle.radian

      rightData.value = {
        x: Math.cos(angleRad) * distance,  // X轴位移
        y: 0,  // Y轴锁定
        distance: distance,
        angle: data.angle.degree
      }

      // 更新速度值
      currentAz = -rightData.value.x  // X轴控制旋转（取反）

      // 第一次移动时启动定时器
      if (sendInterval === null) {
        startSendingVelocity()
      }
    })

    // 监听释放事件
    rightManager.value.on('end', () => {
      console.log('[JOYSTICK] 右摇杆 END 事件触发')
      rightData.value = { x: 0, y: 0, distance: 0, angle: 0 }
      currentAz = 0

      // 如果左摇杆也已释放（速度为0），停止定时器
      if (currentVx === 0 && currentVy === 0) {
        stopSendingVelocity()
      } else {
        // 左摇杆还在使用，继续发送（但 az=0）
        onVelocityChange(currentVx, currentVy, currentAz)
      }
    })
  }

  // 初始化
  onMounted(() => {
    console.log('[JOYSTICK] onMounted 触发，延迟 300ms 创建摇杆')
    // 延迟创建，确保 DOM 完全渲染和布局计算完成（Vue 3 + flexbox）
    setTimeout(() => {
      console.log('[JOYSTICK] 开始创建摇杆...')
      createLeftJoystick()
      createRightJoystick()
    }, 300)  // 从 100ms 增加到 300ms
  })

  // 清理
  onUnmounted(() => {
    if (leftManager.value) {
      leftManager.value.destroy()
    }
    if (rightManager.value) {
      rightManager.value.destroy()
    }
  })

  return {
    leftData,
    rightData
  }
}

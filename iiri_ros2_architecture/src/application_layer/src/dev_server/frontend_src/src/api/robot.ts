/**
 * 机器人 API 接口
 */

export interface DeviceInfo {
  uuid: string
  deviceName?: string
  deviceType?: string
}

/**
 * 获取设备 UUID
 * @returns Promise<string> 设备 UUID
 * @throws Error 如果获取失败
 */
export async function fetchDeviceUUID(): Promise<string> {
  try {
    const response = await fetch('/api/device/uuid')
    if (!response.ok) {
      throw new Error(`HTTP error! status: ${response.status}`)
    }
    const data: DeviceInfo = await response.json()
    return data.uuid
  } catch (error) {
    console.error('Failed to fetch device UUID:', error)
    throw error
  }
}

/**
 * 获取完整的设备信息
 * @returns Promise<DeviceInfo> 设备信息对象
 */
export async function fetchDeviceInfo(): Promise<DeviceInfo> {
  try {
    const response = await fetch('/api/device/info')
    if (!response.ok) {
      throw new Error(`HTTP error! status: ${response.status}`)
    }
    return await response.json()
  } catch (error) {
    console.error('Failed to fetch device info:', error)
    throw error
  }
}

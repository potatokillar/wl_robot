/**
 * @Author: 唐文浩
 * @Date: 2025-11-04
 * @Description: 版本信息 API 接口
 */

/**
 * ROS2 版本信息
 */
export interface ROS2Version {
  architecture?: string
  version?: string
  buildDate?: string
  gitBranch?: string
  gitCommit?: string
  buildHost?: string
  isActive: boolean
  deployPath: string
  source: string
  error?: string
}

/**
 * QR 版本信息
 */
export interface QRVersion {
  architecture?: string
  version?: string
  buildDate?: string
  gitBranch?: string
  gitCommit?: string
  buildHost?: string
  isActive: boolean
  deployPath: string
  source: string
  note?: string
  error?: string
}

/**
 * 版本历史条目
 */
export interface VersionHistoryItem {
  architecture?: string
  version?: string
  buildDate?: string
  gitBranch?: string
  gitCommit?: string
  buildHost?: string
  isActive: boolean
  deployPath: string
  type: 'ros2' | 'qr'
  source?: string
}

/**
 * 版本历史列表
 */
export interface VersionHistory {
  currentSymlink: string
  ros2Versions: VersionHistoryItem[]
  qrVersions: VersionHistoryItem[]
  totalCount: number
  error?: string
}

/**
 * 当前激活版本信息
 */
export interface CurrentVersion {
  ros2: {
    symlinkPath: string
    targetPath: string
  }
  qr: {
    symlinkPath: string
    targetPath: string
  }
  error?: string
}

/**
 * 获取 ROS2 集群版本信息
 * @returns Promise<ROS2Version> ROS2 版本信息
 * @throws Error 如果获取失败
 */
export async function fetchROS2Version(): Promise<ROS2Version> {
  try {
    const response = await fetch('/api/version/ros2')
    if (!response.ok) {
      throw new Error(`HTTP error! status: ${response.status}`)
    }
    return await response.json()
  } catch (error) {
    console.error('Failed to fetch ROS2 version:', error)
    throw error
  }
}

/**
 * 获取 QR 控制进程版本信息
 * @returns Promise<QRVersion> QR 版本信息
 * @throws Error 如果获取失败
 */
export async function fetchQRVersion(): Promise<QRVersion> {
  try {
    const response = await fetch('/api/version/qr')
    if (!response.ok) {
      throw new Error(`HTTP error! status: ${response.status}`)
    }
    return await response.json()
  } catch (error) {
    console.error('Failed to fetch QR version:', error)
    throw error
  }
}

/**
 * 获取版本历史列表
 * @returns Promise<VersionHistory> 版本历史信息
 * @throws Error 如果获取失败
 */
export async function fetchVersionHistory(): Promise<VersionHistory> {
  try {
    const response = await fetch('/api/version/history')
    if (!response.ok) {
      throw new Error(`HTTP error! status: ${response.status}`)
    }
    return await response.json()
  } catch (error) {
    console.error('Failed to fetch version history:', error)
    throw error
  }
}

/**
 * 获取当前激活版本信息
 * @returns Promise<CurrentVersion> 当前激活版本信息
 * @throws Error 如果获取失败
 */
export async function fetchCurrentVersion(): Promise<CurrentVersion> {
  try {
    const response = await fetch('/api/version/current')
    if (!response.ok) {
      throw new Error(`HTTP error! status: ${response.status}`)
    }
    return await response.json()
  } catch (error) {
    console.error('Failed to fetch current version:', error)
    throw error
  }
}

/**
 * 版本切换结果
 */
export interface SwitchResult {
  success: boolean
  message: string
  previousVersion: string
  currentVersion: string
  backupPath: string
  errorDetails?: string
}

/**
 * 版本清理分析结果
 */
export interface CleanupAnalysis {
  totalVersions: number
  versionsToDelete: number
  versionList: string[]
  spaceToFreeMB: number
  activeVersion: string
}

/**
 * 版本清理执行结果
 */
export interface CleanupResult {
  success: boolean
  message: string
  deletedVersions: string[]
  remainingVersions: string[]
  freedSpaceMB: number
  errorDetails?: string
}

/**
 * 切换版本
 * @param type 类型：'ros2' 或 'qr'
 * @param targetVersion 目标版本路径
 * @returns Promise<SwitchResult> 切换结果
 * @throws Error 如果切换失败
 */
export async function switchVersion(type: 'ros2' | 'qr', targetVersion: string): Promise<SwitchResult> {
  try {
    const response = await fetch('/api/version/switch', {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json'
      },
      body: JSON.stringify({
        type,
        targetVersion
      })
    })
    if (!response.ok) {
      throw new Error(`HTTP error! status: ${response.status}`)
    }
    return await response.json()
  } catch (error) {
    console.error('Failed to switch version:', error)
    throw error
  }
}

/**
 * 分析版本清理（不执行删除）
 * @param type 类型：'ros2' 或 'qr'
 * @param keepCount 保留版本数量（默认 5）
 * @returns Promise<CleanupAnalysis> 分析结果
 * @throws Error 如果分析失败
 */
export async function analyzeCleanup(type: 'ros2' | 'qr', keepCount: number = 5): Promise<CleanupAnalysis> {
  try {
    const response = await fetch(`/api/version/cleanup/analyze?type=${type}&keepCount=${keepCount}`)
    if (!response.ok) {
      throw new Error(`HTTP error! status: ${response.status}`)
    }
    return await response.json()
  } catch (error) {
    console.error('Failed to analyze cleanup:', error)
    throw error
  }
}

/**
 * 执行版本清理
 * @param type 类型：'ros2' 或 'qr'
 * @param keepCount 保留版本数量（默认 5）
 * @returns Promise<CleanupResult> 清理结果
 * @throws Error 如果清理失败
 */
export async function cleanupOldVersions(type: 'ros2' | 'qr', keepCount: number = 5): Promise<CleanupResult> {
  try {
    const response = await fetch(`/api/version/cleanup?type=${type}&keepCount=${keepCount}`, {
      method: 'DELETE'
    })
    if (!response.ok) {
      throw new Error(`HTTP error! status: ${response.status}`)
    }
    return await response.json()
  } catch (error) {
    console.error('Failed to cleanup old versions:', error)
    throw error
  }
}

/**
 * 删除指定版本
 * @param type 类型：'ros2' 或 'qr'
 * @param versionPath 版本完整路径
 * @returns Promise<CleanupResult> 删除结果
 * @throws Error 如果删除失败
 */
export async function deleteVersion(type: 'ros2' | 'qr', versionPath: string): Promise<CleanupResult> {
  try {
    const response = await fetch('/api/version/delete', {
      method: 'DELETE',
      headers: {
        'Content-Type': 'application/json'
      },
      body: JSON.stringify({
        type,
        versionPath
      })
    })
    if (!response.ok) {
      throw new Error(`HTTP error! status: ${response.status}`)
    }
    return await response.json()
  } catch (error) {
    console.error('Failed to delete version:', error)
    throw error
  }
}

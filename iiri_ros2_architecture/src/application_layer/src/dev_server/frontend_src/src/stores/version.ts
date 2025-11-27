/**
 * @Author: 唐文浩
 * @Date: 2025-11-04
 * @Description: 版本信息状态管理 Store
 */

import { defineStore } from 'pinia'
import { ref, computed } from 'vue'
import type {
  ROS2Version,
  QRVersion,
  VersionHistory,
  CurrentVersion,
  SwitchResult,
  CleanupAnalysis,
  CleanupResult
} from '@/api/version'
import {
  fetchROS2Version,
  fetchQRVersion,
  fetchVersionHistory,
  fetchCurrentVersion,
  switchVersion,
  analyzeCleanup,
  cleanupOldVersions,
  deleteVersion
} from '@/api/version'

export const useVersionStore = defineStore('version', () => {
  // === 状态 ===

  // ROS2 版本信息
  const ros2Version = ref<ROS2Version | null>(null)
  const ros2Loading = ref(false)
  const ros2Error = ref<string | null>(null)

  // QR 版本信息
  const qrVersion = ref<QRVersion | null>(null)
  const qrLoading = ref(false)
  const qrError = ref<string | null>(null)

  // 版本历史
  const versionHistory = ref<VersionHistory | null>(null)
  const historyLoading = ref(false)
  const historyError = ref<string | null>(null)

  // 当前激活版本
  const currentVersion = ref<CurrentVersion | null>(null)
  const currentLoading = ref(false)
  const currentError = ref<string | null>(null)

  // 最后更新时间
  const lastUpdate = ref<number>(Date.now())

  // 版本切换状态
  const switchResult = ref<SwitchResult | null>(null)
  const switchLoading = ref(false)
  const switchError = ref<string | null>(null)

  // 版本清理分析状态
  const cleanupAnalysis = ref<CleanupAnalysis | null>(null)
  const analysisLoading = ref(false)
  const analysisError = ref<string | null>(null)

  // 版本清理执行状态
  const cleanupResult = ref<CleanupResult | null>(null)
  const cleanupLoading = ref(false)
  const cleanupError = ref<string | null>(null)

  // === 计算属性 ===

  /**
   * 是否有任何加载中的请求
   */
  const isLoading = computed(() => {
    return ros2Loading.value || qrLoading.value || historyLoading.value || currentLoading.value ||
           switchLoading.value || analysisLoading.value || cleanupLoading.value
  })

  /**
   * 是否有任何错误
   */
  const hasError = computed(() => {
    return !!(ros2Error.value || qrError.value || historyError.value || currentError.value ||
              switchError.value || analysisError.value || cleanupError.value)
  })

  /**
   * 汇总的错误信息
   */
  const errorMessage = computed(() => {
    const errors = [ros2Error.value, qrError.value, historyError.value, currentError.value,
                    switchError.value, analysisError.value, cleanupError.value]
    return errors.filter(e => e !== null).join('; ')
  })

  // === Actions ===

  /**
   * 加载 ROS2 版本信息
   */
  async function loadROS2Version() {
    ros2Loading.value = true
    ros2Error.value = null
    try {
      ros2Version.value = await fetchROS2Version()
      lastUpdate.value = Date.now()
    } catch (error) {
      ros2Error.value = error instanceof Error ? error.message : '加载 ROS2 版本失败'
      console.error('Failed to load ROS2 version:', error)
    } finally {
      ros2Loading.value = false
    }
  }

  /**
   * 加载 QR 版本信息
   */
  async function loadQRVersion() {
    qrLoading.value = true
    qrError.value = null
    try {
      qrVersion.value = await fetchQRVersion()
      lastUpdate.value = Date.now()
    } catch (error) {
      qrError.value = error instanceof Error ? error.message : '加载 QR 版本失败'
      console.error('Failed to load QR version:', error)
    } finally {
      qrLoading.value = false
    }
  }

  /**
   * 加载版本历史列表
   */
  async function loadVersionHistory() {
    historyLoading.value = true
    historyError.value = null
    try {
      versionHistory.value = await fetchVersionHistory()
      lastUpdate.value = Date.now()
    } catch (error) {
      historyError.value = error instanceof Error ? error.message : '加载版本历史失败'
      console.error('Failed to load version history:', error)
    } finally {
      historyLoading.value = false
    }
  }

  /**
   * 加载当前激活版本信息
   */
  async function loadCurrentVersion() {
    currentLoading.value = true
    currentError.value = null
    try {
      currentVersion.value = await fetchCurrentVersion()
      lastUpdate.value = Date.now()
    } catch (error) {
      currentError.value = error instanceof Error ? error.message : '加载当前版本失败'
      console.error('Failed to load current version:', error)
    } finally {
      currentLoading.value = false
    }
  }

  /**
   * 加载所有版本信息
   */
  async function loadAllVersions() {
    await Promise.all([
      loadROS2Version(),
      loadQRVersion(),
      loadVersionHistory(),
      loadCurrentVersion()
    ])
  }

  /**
   * 切换版本
   * @param type 类型：'ros2' 或 'qr'
   * @param targetVersion 目标版本路径
   */
  async function performVersionSwitch(type: 'ros2' | 'qr', targetVersion: string) {
    switchLoading.value = true
    switchError.value = null
    switchResult.value = null
    try {
      switchResult.value = await switchVersion(type, targetVersion)
      lastUpdate.value = Date.now()
      // 切换成功后重新加载所有版本信息
      if (switchResult.value.success) {
        await loadAllVersions()
      }
    } catch (error) {
      switchError.value = error instanceof Error ? error.message : '版本切换失败'
      console.error('Failed to switch version:', error)
    } finally {
      switchLoading.value = false
    }
  }

  /**
   * 分析版本清理（不执行删除）
   * @param type 类型：'ros2' 或 'qr'
   * @param keepCount 保留版本数量
   */
  async function performCleanupAnalysis(type: 'ros2' | 'qr', keepCount: number = 5) {
    analysisLoading.value = true
    analysisError.value = null
    cleanupAnalysis.value = null
    try {
      cleanupAnalysis.value = await analyzeCleanup(type, keepCount)
      lastUpdate.value = Date.now()
    } catch (error) {
      analysisError.value = error instanceof Error ? error.message : '清理分析失败'
      console.error('Failed to analyze cleanup:', error)
    } finally {
      analysisLoading.value = false
    }
  }

  /**
   * 执行版本清理
   * @param type 类型：'ros2' 或 'qr'
   * @param keepCount 保留版本数量
   */
  async function performCleanup(type: 'ros2' | 'qr', keepCount: number = 5) {
    cleanupLoading.value = true
    cleanupError.value = null
    cleanupResult.value = null
    try {
      cleanupResult.value = await cleanupOldVersions(type, keepCount)
      lastUpdate.value = Date.now()
      // 清理成功后重新加载版本历史
      if (cleanupResult.value.success) {
        await loadVersionHistory()
      }
    } catch (error) {
      cleanupError.value = error instanceof Error ? error.message : '版本清理失败'
      console.error('Failed to cleanup old versions:', error)
    } finally {
      cleanupLoading.value = false
    }
  }

  /**
   * 删除指定版本
   * @param type 类型：'ros2' 或 'qr'
   * @param versionPath 版本完整路径
   */
  async function performDeleteVersion(type: 'ros2' | 'qr', versionPath: string) {
    cleanupLoading.value = true
    cleanupError.value = null
    cleanupResult.value = null
    try {
      cleanupResult.value = await deleteVersion(type, versionPath)
      lastUpdate.value = Date.now()
      // 删除成功后重新加载版本历史
      if (cleanupResult.value.success) {
        await loadVersionHistory()
      }
    } catch (error) {
      cleanupError.value = error instanceof Error ? error.message : '版本删除失败'
      console.error('Failed to delete version:', error)
    } finally {
      cleanupLoading.value = false
    }
  }

  /**
   * 清除所有错误
   */
  function clearErrors() {
    ros2Error.value = null
    qrError.value = null
    historyError.value = null
    currentError.value = null
    switchError.value = null
    analysisError.value = null
    cleanupError.value = null
  }

  /**
   * 重置所有状态
   */
  function reset() {
    ros2Version.value = null
    ros2Loading.value = false
    ros2Error.value = null

    qrVersion.value = null
    qrLoading.value = false
    qrError.value = null

    versionHistory.value = null
    historyLoading.value = false
    historyError.value = null

    currentVersion.value = null
    currentLoading.value = false
    currentError.value = null

    switchResult.value = null
    switchLoading.value = false
    switchError.value = null

    cleanupAnalysis.value = null
    analysisLoading.value = false
    analysisError.value = null

    cleanupResult.value = null
    cleanupLoading.value = false
    cleanupError.value = null

    lastUpdate.value = Date.now()
  }

  return {
    // 状态
    ros2Version,
    ros2Loading,
    ros2Error,
    qrVersion,
    qrLoading,
    qrError,
    versionHistory,
    historyLoading,
    historyError,
    currentVersion,
    currentLoading,
    currentError,
    switchResult,
    switchLoading,
    switchError,
    cleanupAnalysis,
    analysisLoading,
    analysisError,
    cleanupResult,
    cleanupLoading,
    cleanupError,
    lastUpdate,

    // 计算属性
    isLoading,
    hasError,
    errorMessage,

    // 方法
    loadROS2Version,
    loadQRVersion,
    loadVersionHistory,
    loadCurrentVersion,
    loadAllVersions,
    performVersionSwitch,
    performCleanupAnalysis,
    performCleanup,
    performDeleteVersion,
    clearErrors,
    reset
  }
})

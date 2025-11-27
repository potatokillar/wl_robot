/**
 * @Author: 唐文浩
 * @Date: 2025-11-03
 * @Description: OTA 更新 API 接口
 */

import axios from 'axios'

// 使用相对路径，避免跨域问题
const API_BASE_URL = import.meta.env.VITE_API_BASE_URL || ''

export interface UploadResponse {
  success: boolean
  taskId: string
  message: string
  packagePath?: string
  sha256Path?: string
  appType: string
}

export interface StartUpdateRequest {
  taskId: string
  appType: string
  daemon: boolean
}

export interface StartUpdateResponse {
  success: boolean
  taskId: string
  message: string
  pid?: number
  statusFile?: string
}

export interface UpdateStatus {
  taskId: string
  state: string
  progress: number
  message: string
  errorMsg?: string
  startTime?: string
  endTime?: string
  duration?: number
}

export interface StatusResponse {
  success: boolean
  status?: UpdateStatus
  message?: string
}

export interface CancelUpdateResponse {
  success: boolean
  message: string
}

/**
 * 上传更新包和 SHA256 校验文件
 * @param formData 包含 file, sha256_file, app_type 的表单数据
 * @param onProgress 上传进度回调函数，参数为进度百分比 (0-100)
 * @returns 上传结果
 */
export const uploadUpdatePackage = async (
  formData: FormData,
  onProgress?: (progress: number) => void
): Promise<UploadResponse> => {
  try {
    console.log('发送上传请求到:', `${API_BASE_URL}/api/update/upload`)
    const response = await axios.post(`${API_BASE_URL}/api/update/upload`, formData, {
      headers: {
        'Content-Type': 'multipart/form-data'
      },
      timeout: 300000, // 5分钟超时
      onUploadProgress: (progressEvent) => {
        if (progressEvent.total && onProgress) {
          const percentCompleted = Math.round((progressEvent.loaded * 100) / progressEvent.total)
          console.log(`上传进度: ${percentCompleted}%`)
          onProgress(percentCompleted)
        }
      }
    })
    console.log('上传响应:', response.data)
    return response.data
  } catch (error: any) {
    console.error('Upload failed:', error)
    console.error('错误详情:', {
      status: error.response?.status,
      statusText: error.response?.statusText,
      data: error.response?.data,
      message: error.message
    })
    throw new Error(error.response?.data?.message || error.message || 'Upload failed')
  }
}

/**
 * 启动更新任务
 * @param data 更新任务参数
 * @returns 启动结果
 */
export const startUpdateTask = async (data: StartUpdateRequest): Promise<StartUpdateResponse> => {
  try {
    const response = await axios.post(`${API_BASE_URL}/api/update/start`, data)
    return response.data
  } catch (error: any) {
    console.error('Start update failed:', error)
    throw new Error(error.response?.data?.message || error.message || 'Start update failed')
  }
}

/**
 * 查询更新状态
 * @param taskId 任务ID
 * @returns 状态信息
 */
export const getUpdateStatus = async (taskId: string): Promise<StatusResponse> => {
  try {
    const response = await axios.get(`${API_BASE_URL}/api/update/status/${taskId}`)
    return response.data
  } catch (error: any) {
    console.error('Get status failed:', error)
    throw new Error(error.response?.data?.message || error.message || 'Get status failed')
  }
}

/**
 * 取消更新任务
 * @param taskId 任务ID
 * @returns 取消结果
 */
export const cancelUpdateTask = async (taskId: string): Promise<CancelUpdateResponse> => {
  try {
    const response = await axios.post(`${API_BASE_URL}/api/update/cancel/${taskId}`)
    return response.data
  } catch (error: any) {
    console.error('Cancel update failed:', error)
    throw new Error(error.response?.data?.message || error.message || 'Cancel update failed')
  }
}

/**
 * 获取当前更新状态（直接读取状态文件，用于 WebSocket 断开后的轮询）
 * @returns 当前更新状态
 */
export const getCurrentStatus = async (): Promise<StatusResponse> => {
  try {
    const response = await axios.get(`${API_BASE_URL}/api/update/current-status`, {
      timeout: 5000 // 5秒超时
    })
    return response.data
  } catch (error: any) {
    console.error('Get current status failed:', error)
    throw new Error(error.response?.data?.message || error.message || 'Get current status failed')
  }
}

<!--
 * @Author: 唐文浩
 * @Date: 2025-11-04
 * @Description: 系统版本信息页面 - 增强版（支持版本切换和清理）
 -->

<template>
  <div class="version-view">
    <el-card>
      <template #header>
        <div class="card-header">
          <span>系统版本信息</span>
          <div class="header-actions">
            <el-button
              type="danger"
              :loading="versionStore.cleanupLoading"
              @click="showCleanupDialog"
              size="small"
              plain
            >
              清理旧版本
            </el-button>
            <el-button
              type="primary"
              :loading="versionStore.isLoading"
              @click="refreshAll"
              size="small"
            >
              刷新
            </el-button>
          </div>
        </div>
      </template>

      <!-- 错误提示 -->
      <el-alert
        v-if="versionStore.hasError"
        :title="versionStore.errorMessage"
        type="error"
        :closable="false"
        style="margin-bottom: 20px;"
      />

      <!-- 当前版本信息 -->
      <el-row :gutter="20" style="margin-bottom: 20px;">
        <!-- ROS2 版本卡片 -->
        <el-col :xs="24" :sm="24" :md="12" :lg="12" :xl="12">
          <el-card shadow="hover">
            <template #header>
              <div class="version-card-header">
                <span>ROS2 集群版本</span>
                <el-tag v-if="versionStore.ros2Version?.isActive" type="success" size="small">
                  激活中
                </el-tag>
              </div>
            </template>
            <div v-if="versionStore.ros2Loading" class="loading-container">
              <el-skeleton :rows="4" animated />
            </div>
            <div v-else-if="versionStore.ros2Version" class="version-info">
              <el-descriptions :column="1" border size="small">
                <el-descriptions-item label="版本号">
                  {{ versionStore.ros2Version.version || 'N/A' }}
                </el-descriptions-item>
                <el-descriptions-item label="架构">
                  {{ versionStore.ros2Version.architecture || 'N/A' }}
                </el-descriptions-item>
                <el-descriptions-item label="构建日期">
                  {{ versionStore.ros2Version.buildDate || 'N/A' }}
                </el-descriptions-item>
                <el-descriptions-item label="Git 分支">
                  {{ versionStore.ros2Version.gitBranch || 'N/A' }}
                </el-descriptions-item>
                <el-descriptions-item label="Git Commit">
                  <el-text type="info" size="small">
                    {{ versionStore.ros2Version.gitCommit || 'N/A' }}
                  </el-text>
                </el-descriptions-item>
                <el-descriptions-item label="构建主机">
                  {{ versionStore.ros2Version.buildHost || 'N/A' }}
                </el-descriptions-item>
                <el-descriptions-item label="部署路径">
                  <el-text type="info" size="small">
                    {{ versionStore.ros2Version.deployPath }}
                  </el-text>
                </el-descriptions-item>
              </el-descriptions>
            </div>
            <el-empty v-else description="无法加载版本信息" />
          </el-card>
        </el-col>

        <!-- QR 版本卡片 -->
        <el-col :xs="24" :sm="24" :md="12" :lg="12" :xl="12">
          <el-card shadow="hover">
            <template #header>
              <div class="version-card-header">
                <span>QR 控制进程版本</span>
                <el-tag v-if="versionStore.qrVersion?.isActive" type="success" size="small">
                  激活中
                </el-tag>
              </div>
            </template>
            <div v-if="versionStore.qrLoading" class="loading-container">
              <el-skeleton :rows="4" animated />
            </div>
            <div v-else-if="versionStore.qrVersion && !versionStore.qrVersion.note" class="version-info">
              <el-descriptions :column="1" border size="small">
                <el-descriptions-item label="版本号">
                  {{ versionStore.qrVersion.version || 'N/A' }}
                </el-descriptions-item>
                <el-descriptions-item label="架构">
                  {{ versionStore.qrVersion.architecture || 'N/A' }}
                </el-descriptions-item>
                <el-descriptions-item label="构建日期">
                  {{ versionStore.qrVersion.buildDate || 'N/A' }}
                </el-descriptions-item>
                <el-descriptions-item label="Git 分支">
                  {{ versionStore.qrVersion.gitBranch || 'N/A' }}
                </el-descriptions-item>
                <el-descriptions-item label="Git Commit">
                  <el-text type="info" size="small">
                    {{ versionStore.qrVersion.gitCommit || 'N/A' }}
                  </el-text>
                </el-descriptions-item>
                <el-descriptions-item label="构建主机">
                  {{ versionStore.qrVersion.buildHost || 'N/A' }}
                </el-descriptions-item>
                <el-descriptions-item label="部署路径">
                  <el-text type="info" size="small">
                    {{ versionStore.qrVersion.deployPath }}
                  </el-text>
                </el-descriptions-item>
              </el-descriptions>
            </div>
            <div v-else-if="versionStore.qrVersion && versionStore.qrVersion.note" class="version-note">
              <el-result :title="versionStore.qrVersion.note" icon="warning" />
            </div>
            <el-empty v-else description="无法加载版本信息" />
          </el-card>
        </el-col>
      </el-row>

      <!-- 版本历史和操作 -->
      <el-card shadow="never">
        <template #header>
          <span>版本历史</span>
        </template>
        <div v-if="versionStore.historyLoading" class="loading-container">
          <el-skeleton :rows="5" animated />
        </div>
        <div v-else-if="versionStore.versionHistory">
          <!-- ROS2 版本历史 -->
          <el-divider content-position="left">
            <el-text type="primary" size="large">ROS2 版本历史</el-text>
          </el-divider>

          <!-- 桌面端：表格布局 -->
          <el-table
            v-if="!isMobile"
            :data="versionStore.versionHistory.ros2Versions"
            stripe
            style="width: 100%; margin-bottom: 20px;"
            :empty-text="'暂无 ROS2 版本历史'"
          >
            <el-table-column label="状态" width="80" align="center">
              <template #default="{ row }">
                <el-tag v-if="row.isActive" type="success" size="small">激活</el-tag>
                <el-tag v-else type="info" size="small">历史</el-tag>
              </template>
            </el-table-column>
            <el-table-column prop="version" label="版本号" width="180" />
            <el-table-column prop="architecture" label="架构" width="100" />
            <el-table-column prop="buildDate" label="构建日期" width="180" />
            <el-table-column prop="gitBranch" label="Git 分支" width="120" />
            <el-table-column prop="gitCommit" label="Git Commit" width="150">
              <template #default="{ row }">
                <el-text type="info" size="small">{{ row.gitCommit || 'N/A' }}</el-text>
              </template>
            </el-table-column>
            <el-table-column prop="deployPath" label="部署路径" min-width="200" show-overflow-tooltip />
            <el-table-column label="操作" width="180" align="center">
              <template #default="{ row }">
                <el-button
                  v-if="!row.isActive"
                  type="primary"
                  size="small"
                  @click="handleVersionSwitch('ros2', row.deployPath)"
                  :loading="versionStore.switchLoading"
                >
                  切换
                </el-button>
                <el-button
                  v-if="!row.isActive"
                  type="danger"
                  size="small"
                  @click="handleVersionDelete('ros2', row.deployPath)"
                  :loading="versionStore.cleanupLoading"
                  plain
                >
                  删除
                </el-button>
              </template>
            </el-table-column>
          </el-table>

          <!-- 移动端：卡片布局 -->
          <div v-else class="mobile-version-list">
            <el-empty v-if="versionStore.versionHistory.ros2Versions.length === 0" description="暂无 ROS2 版本历史" />
            <el-card
              v-for="version in versionStore.versionHistory.ros2Versions"
              :key="version.deployPath"
              class="mobile-version-card"
              shadow="hover"
            >
              <div class="card-header-mobile">
                <el-tag :type="version.isActive ? 'success' : 'info'" size="small">
                  {{ version.isActive ? '当前激活' : '历史版本' }}
                </el-tag>
                <span class="version-number">{{ version.version }}</span>
              </div>

              <div class="card-content-mobile">
                <div class="info-row">
                  <span class="label">构建日期:</span>
                  <span class="value">{{ version.buildDate }}</span>
                </div>
                <div class="info-row">
                  <span class="label">架构:</span>
                  <span class="value">{{ version.architecture }}</span>
                </div>

                <el-collapse accordion>
                  <el-collapse-item title="详细信息">
                    <div class="info-row">
                      <span class="label">Git 分支:</span>
                      <span class="value">{{ version.gitBranch }}</span>
                    </div>
                    <div class="info-row">
                      <span class="label">Git Commit:</span>
                      <span class="value">{{ version.gitCommit || 'N/A' }}</span>
                    </div>
                    <div class="info-row">
                      <span class="label">部署路径:</span>
                      <span class="value">{{ version.deployPath }}</span>
                    </div>
                  </el-collapse-item>
                </el-collapse>
              </div>

              <div v-if="!version.isActive" class="card-actions-mobile">
                <el-button
                  type="primary"
                  size="small"
                  @click="handleVersionSwitch('ros2', version.deployPath)"
                  :loading="versionStore.switchLoading"
                >
                  切换
                </el-button>
                <el-button
                  type="danger"
                  size="small"
                  @click="handleVersionDelete('ros2', version.deployPath)"
                  :loading="versionStore.cleanupLoading"
                  plain
                >
                  删除
                </el-button>
              </div>
            </el-card>
          </div>

          <!-- QR 版本历史 -->
          <el-divider content-position="left">
            <el-text type="primary" size="large">QR 版本历史</el-text>
          </el-divider>

          <!-- 桌面端：表格布局 -->
          <el-table
            v-if="!isMobile"
            :data="versionStore.versionHistory.qrVersions"
            stripe
            style="width: 100%;"
            :empty-text="'暂无 QR 版本历史'"
          >
            <el-table-column label="状态" width="80" align="center">
              <template #default="{ row }">
                <el-tag v-if="row.isActive" type="success" size="small">激活</el-tag>
                <el-tag v-else type="info" size="small">历史</el-tag>
              </template>
            </el-table-column>
            <el-table-column prop="version" label="版本号" width="180" />
            <el-table-column prop="architecture" label="架构" width="100" />
            <el-table-column prop="buildDate" label="构建日期" width="180" />
            <el-table-column prop="gitBranch" label="Git 分支" width="120" />
            <el-table-column prop="gitCommit" label="Git Commit" width="150">
              <template #default="{ row }">
                <el-text type="info" size="small">{{ row.gitCommit || 'N/A' }}</el-text>
              </template>
            </el-table-column>
            <el-table-column prop="deployPath" label="部署路径" min-width="200" show-overflow-tooltip />
            <el-table-column label="操作" width="180" align="center">
              <template #default="{ row }">
                <el-button
                  v-if="!row.isActive"
                  type="primary"
                  size="small"
                  @click="handleVersionSwitch('qr', row.deployPath)"
                  :loading="versionStore.switchLoading"
                >
                  切换
                </el-button>
                <el-button
                  v-if="!row.isActive"
                  type="danger"
                  size="small"
                  @click="handleVersionDelete('qr', row.deployPath)"
                  :loading="versionStore.cleanupLoading"
                  plain
                >
                  删除
                </el-button>
              </template>
            </el-table-column>
          </el-table>

          <!-- 移动端：卡片布局 -->
          <div v-else class="mobile-version-list">
            <el-empty v-if="versionStore.versionHistory.qrVersions.length === 0" description="暂无 QR 版本历史" />
            <el-card
              v-for="version in versionStore.versionHistory.qrVersions"
              :key="version.deployPath"
              class="mobile-version-card"
              shadow="hover"
            >
              <div class="card-header-mobile">
                <el-tag :type="version.isActive ? 'success' : 'info'" size="small">
                  {{ version.isActive ? '当前激活' : '历史版本' }}
                </el-tag>
                <span class="version-number">{{ version.version }}</span>
              </div>

              <div class="card-content-mobile">
                <div class="info-row">
                  <span class="label">构建日期:</span>
                  <span class="value">{{ version.buildDate }}</span>
                </div>
                <div class="info-row">
                  <span class="label">架构:</span>
                  <span class="value">{{ version.architecture }}</span>
                </div>

                <el-collapse accordion>
                  <el-collapse-item title="详细信息">
                    <div class="info-row">
                      <span class="label">Git 分支:</span>
                      <span class="value">{{ version.gitBranch }}</span>
                    </div>
                    <div class="info-row">
                      <span class="label">Git Commit:</span>
                      <span class="value">{{ version.gitCommit || 'N/A' }}</span>
                    </div>
                    <div class="info-row">
                      <span class="label">部署路径:</span>
                      <span class="value">{{ version.deployPath }}</span>
                    </div>
                  </el-collapse-item>
                </el-collapse>
              </div>

              <div v-if="!version.isActive" class="card-actions-mobile">
                <el-button
                  type="primary"
                  size="small"
                  @click="handleVersionSwitch('qr', version.deployPath)"
                  :loading="versionStore.switchLoading"
                >
                  切换
                </el-button>
                <el-button
                  type="danger"
                  size="small"
                  @click="handleVersionDelete('qr', version.deployPath)"
                  :loading="versionStore.cleanupLoading"
                  plain
                >
                  删除
                </el-button>
              </div>
            </el-card>
          </div>

          <!-- 统计信息 -->
          <el-divider />
          <div class="statistics">
            <el-text type="info">
              共计 {{ versionStore.versionHistory.totalCount }} 个版本
              （ROS2: {{ versionStore.versionHistory.ros2Versions.length }},
              QR: {{ versionStore.versionHistory.qrVersions.length }}）
            </el-text>
          </div>
        </div>
        <el-empty v-else description="无法加载版本历史" />
      </el-card>
    </el-card>

    <!-- 版本清理对话框 -->
    <el-dialog
      v-model="cleanupDialogVisible"
      title="清理旧版本"
      :width="isMobile ? '95%' : '600px'"
      :fullscreen="isMobile"
      :close-on-click-modal="false"
    >
      <div v-if="!cleanupAnalysisData">
        <el-form
          :model="cleanupForm"
          :label-width="isMobile ? 'auto' : '100px'"
          :label-position="isMobile ? 'top' : 'left'"
        >
          <el-form-item label="清理类型">
            <el-radio-group v-model="cleanupForm.type">
              <el-radio label="ros2">ROS2 集群</el-radio>
              <el-radio label="qr">QR 控制进程</el-radio>
            </el-radio-group>
          </el-form-item>
          <el-form-item label="保留数量">
            <el-input-number
              v-model="cleanupForm.keepCount"
              :min="1"
              :max="10"
              controls-position="right"
            />
            <el-text type="info" size="small" style="margin-left: 10px;">
              保留最近的 N 个版本（包括激活版本）
            </el-text>
          </el-form-item>
        </el-form>
      </div>
      <div v-else class="cleanup-analysis">
        <el-descriptions :column="1" border>
          <el-descriptions-item label="清理类型">
            {{ cleanupForm.type === 'ros2' ? 'ROS2 集群' : 'QR 控制进程' }}
          </el-descriptions-item>
          <el-descriptions-item label="总版本数">
            {{ cleanupAnalysisData.totalVersions }}
          </el-descriptions-item>
          <el-descriptions-item label="将要删除">
            {{ cleanupAnalysisData.versionsToDelete }} 个版本
          </el-descriptions-item>
          <el-descriptions-item label="释放空间">
            {{ cleanupAnalysisData.spaceToFreeMB.toFixed(2) }} MB
          </el-descriptions-item>
          <el-descriptions-item label="激活版本">
            <el-text type="success" size="small">{{ cleanupAnalysisData.activeVersion }}</el-text>
          </el-descriptions-item>
        </el-descriptions>
        <el-divider />
        <el-text type="warning" size="small">将删除以下版本：</el-text>
        <ul class="version-list">
          <li v-for="version in cleanupAnalysisData.versionList" :key="version">
            {{ version }}
          </li>
        </ul>
      </div>
      <template #footer>
        <el-button @click="cleanupDialogVisible = false">取消</el-button>
        <el-button
          v-if="!cleanupAnalysisData"
          type="primary"
          @click="analyzeCleanup"
          :loading="versionStore.analysisLoading"
        >
          分析
        </el-button>
        <el-button
          v-else
          type="danger"
          @click="executeCleanup"
          :loading="versionStore.cleanupLoading"
        >
          确认清理
        </el-button>
      </template>
    </el-dialog>
  </div>
</template>

<script setup lang="ts">
import { ref, onMounted, onUnmounted } from 'vue'
import { useVersionStore } from '@/stores/version'
import { ElMessage, ElMessageBox } from 'element-plus'
import type { CleanupAnalysis } from '@/api/version'

const versionStore = useVersionStore()

// 移动端检测
const isMobile = ref(false)
const isTablet = ref(false)

const updateDeviceType = () => {
  const width = window.innerWidth
  isMobile.value = width <= 768
  isTablet.value = width > 768 && width <= 1024
}

// 清理对话框状态
const cleanupDialogVisible = ref(false)
const cleanupForm = ref({
  type: 'ros2' as 'ros2' | 'qr',
  keepCount: 5
})
const cleanupAnalysisData = ref<CleanupAnalysis | null>(null)

/**
 * 刷新所有版本信息
 */
async function refreshAll() {
  try {
    await versionStore.loadAllVersions()
    ElMessage.success('版本信息已刷新')
  } catch (error) {
    ElMessage.error('刷新版本信息失败')
    console.error('Failed to refresh versions:', error)
  }
}

/**
 * 显示清理对话框
 */
function showCleanupDialog() {
  cleanupDialogVisible.value = true
  cleanupAnalysisData.value = null
  cleanupForm.value = {
    type: 'ros2',
    keepCount: 5
  }
}

/**
 * 分析版本清理
 */
async function analyzeCleanup() {
  try {
    await versionStore.performCleanupAnalysis(cleanupForm.value.type, cleanupForm.value.keepCount)
    if (versionStore.cleanupAnalysis) {
      cleanupAnalysisData.value = versionStore.cleanupAnalysis
      if (cleanupAnalysisData.value.versionsToDelete === 0) {
        ElMessage.info('没有需要清理的旧版本')
      }
    }
  } catch (error) {
    ElMessage.error('分析清理失败')
    console.error('Failed to analyze cleanup:', error)
  }
}

/**
 * 执行版本清理
 */
async function executeCleanup() {
  try {
    await ElMessageBox.confirm(
      `确定要删除 ${cleanupAnalysisData.value?.versionsToDelete} 个版本吗？此操作不可撤销！`,
      '确认清理',
      {
        confirmButtonText: '确定',
        cancelButtonText: '取消',
        type: 'warning'
      }
    )

    await versionStore.performCleanup(cleanupForm.value.type, cleanupForm.value.keepCount)
    if (versionStore.cleanupResult?.success) {
      ElMessage.success(
        `清理成功！删除 ${versionStore.cleanupResult.deletedVersions.length} 个版本，` +
        `释放 ${versionStore.cleanupResult.freedSpaceMB.toFixed(2)} MB 空间`
      )
      cleanupDialogVisible.value = false
      await refreshAll()
    } else {
      ElMessage.error(`清理失败：${versionStore.cleanupResult?.message || '未知错误'}`)
    }
  } catch (error) {
    if (error !== 'cancel') {
      ElMessage.error('清理操作失败')
      console.error('Failed to cleanup:', error)
    }
  }
}

/**
 * 处理版本切换
 */
async function handleVersionSwitch(type: 'ros2' | 'qr', targetVersion: string) {
  try {
    await ElMessageBox.confirm(
      `确定要切换到版本 ${targetVersion} 吗？服务将自动重启。`,
      '确认切换',
      {
        confirmButtonText: '确定',
        cancelButtonText: '取消',
        type: 'warning'
      }
    )

    await versionStore.performVersionSwitch(type, targetVersion)
    if (versionStore.switchResult?.success) {
      ElMessage.success(
        `版本切换成功！从 ${versionStore.switchResult.previousVersion} 切换到 ${versionStore.switchResult.currentVersion}`
      )
      await refreshAll()
    } else {
      ElMessage.error(`版本切换失败：${versionStore.switchResult?.message || '未知错误'}`)
    }
  } catch (error) {
    if (error !== 'cancel') {
      ElMessage.error('版本切换操作失败')
      console.error('Failed to switch version:', error)
    }
  }
}

/**
 * 处理版本删除
 */
async function handleVersionDelete(type: 'ros2' | 'qr', versionPath: string) {
  try {
    await ElMessageBox.confirm(
      `确定要删除版本 ${versionPath} 吗？此操作不可撤销！`,
      '确认删除',
      {
        confirmButtonText: '确定',
        cancelButtonText: '取消',
        type: 'warning'
      }
    )

    await versionStore.performDeleteVersion(type, versionPath)
    if (versionStore.cleanupResult?.success) {
      ElMessage.success(
        `版本删除成功！释放 ${versionStore.cleanupResult.freedSpaceMB.toFixed(2)} MB 空间`
      )
      await refreshAll()
    } else {
      ElMessage.error(`版本删除失败：${versionStore.cleanupResult?.message || '未知错误'}`)
    }
  } catch (error) {
    if (error !== 'cancel') {
      ElMessage.error('版本删除操作失败')
      console.error('Failed to delete version:', error)
    }
  }
}

/**
 * 组件挂载时自动加载
 */
onMounted(() => {
  updateDeviceType()
  window.addEventListener('resize', updateDeviceType)
  refreshAll()
})

onUnmounted(() => {
  window.removeEventListener('resize', updateDeviceType)
})
</script>

<style scoped>
.version-view {
  padding: 20px;
  width: 100%;
  max-width: 1600px;
  margin: 0 auto;
}

.card-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
}

.header-actions {
  display: flex;
  gap: 10px;
}

.version-card-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  font-weight: bold;
}

.version-info {
  font-size: 14px;
}

.version-note {
  padding: 20px 0;
}

.loading-container {
  padding: 20px;
}

.statistics {
  text-align: center;
  padding: 10px 0;
}

.cleanup-analysis {
  margin-top: 20px;
}

.version-list {
  margin-top: 10px;
  padding-left: 20px;
  max-height: 200px;
  overflow-y: auto;
}

.version-list li {
  padding: 5px 0;
  font-size: 14px;
  color: #606266;
}

:deep(.el-descriptions__label) {
  font-weight: 500;
  width: 120px;
}

:deep(.el-descriptions__content) {
  word-break: break-all;
}

/* ===========================
   移动端优化样式
   =========================== */

@media screen and (max-width: 768px) {
  .version-view {
    padding: 12px;
    max-width: 100%;
  }

  /* 卡片头部优化 */
  .card-header {
    flex-direction: column;
    align-items: flex-start !important;
    gap: 10px;
  }

  .header-actions {
    width: 100%;
    display: flex;
    flex-direction: row;  /* 改为水平排列 */
    gap: 8px;
  }

  .header-actions .el-button {
    flex: 1;  /* 每个按钮平分空间 */
    min-height: 44px;
  }

  /* 版本卡片 */
  .version-card-header {
    flex-direction: column;
    align-items: flex-start !important;
    gap: 8px;
  }

  /* Descriptions 组件压缩 */
  :deep(.el-descriptions__label) {
    width: 80px !important;
    font-size: 12px;
  }

  :deep(.el-descriptions__content) {
    font-size: 12px;
  }

  /* 移动端卡片样式 */
  .mobile-version-list {
    margin-bottom: 20px;
  }

  .mobile-version-card {
    margin-bottom: 12px;
  }

  .mobile-version-card:last-child {
    margin-bottom: 0;
  }

  .card-header-mobile {
    display: flex;
    justify-content: space-between;
    align-items: center;
    margin-bottom: 12px;
    padding-bottom: 12px;
    border-bottom: 1px solid #ebeef5;
  }

  .card-header-mobile .version-number {
    font-size: 16px;
    font-weight: 600;
    color: #303133;
  }

  .card-content-mobile {
    margin-bottom: 12px;
  }

  .card-content-mobile .info-row {
    display: flex;
    justify-content: space-between;
    padding: 8px 0;
    border-bottom: 1px solid #f5f7fa;
  }

  .card-content-mobile .info-row:last-child {
    border-bottom: none;
  }

  .card-content-mobile .label {
    color: #909399;
    font-size: 13px;
    flex-shrink: 0;
    min-width: 70px;
  }

  .card-content-mobile .value {
    color: #303133;
    font-size: 13px;
    text-align: right;
    flex: 1;
    word-break: break-all;
  }

  /* 折叠面板优化 */
  .card-content-mobile :deep(.el-collapse) {
    margin-top: 8px;
    border-top: none;
    border-bottom: none;
  }

  .card-content-mobile :deep(.el-collapse-item__header) {
    background-color: #f5f7fa;
    padding: 8px 12px;
    font-size: 13px;
    height: auto;
    line-height: 1.5;
  }

  .card-content-mobile :deep(.el-collapse-item__content) {
    padding: 12px;
  }

  /* 操作按钮 */
  .card-actions-mobile {
    display: flex;
    gap: 8px;
    padding-top: 12px;
    border-top: 1px solid #ebeef5;
  }

  .card-actions-mobile .el-button {
    flex: 1;
    min-height: 44px;
    font-size: 14px;
  }

  /* 统计信息 */
  .statistics {
    font-size: 13px;
  }

  /* 对话框移动端优化 */
  :deep(.el-dialog__header) {
    padding: 16px;
  }

  :deep(.el-dialog__body) {
    padding: 16px;
  }

  :deep(.el-dialog__footer) {
    padding: 12px 16px;
  }

  :deep(.el-dialog__footer .el-button) {
    width: 100%;
    margin: 4px 0;
  }

  /* 清理分析对话框 */
  .cleanup-analysis {
    margin-top: 0;
  }

  .version-list {
    max-height: 150px;
    font-size: 13px;
  }
}

/* 小屏手机优化 (iPhone SE 等) */
@media screen and (max-width: 480px) {
  .version-view {
    padding: 8px;
  }

  .mobile-version-card {
    margin-bottom: 10px;
  }

  .card-header-mobile .version-number {
    font-size: 14px;
  }

  .card-content-mobile .label,
  .card-content-mobile .value {
    font-size: 12px;
  }

  .card-actions-mobile .el-button {
    font-size: 13px;
    padding: 10px 12px;
  }

  :deep(.el-descriptions__label),
  :deep(.el-descriptions__content) {
    font-size: 11px;
  }
}

/* 横屏模式优化 */
@media screen and (max-width: 768px) and (orientation: landscape) {
  .version-view {
    padding: 8px 16px;
  }

  .mobile-version-card {
    margin-bottom: 8px;
  }

  .card-header-mobile {
    margin-bottom: 8px;
    padding-bottom: 8px;
  }

  .card-content-mobile {
    margin-bottom: 8px;
  }

  .card-actions-mobile {
    padding-top: 8px;
  }
}

/* ===========================
   统一工业风格按钮样式
   =========================== */

/* 刷新按钮（统一蓝色风格） */
.header-actions :deep(.el-button--primary) {
  background: linear-gradient(145deg, #34495e 0%, #2c3e50 100%);
  border: 1px solid #3498db;
  color: #ecf0f1;
  font-weight: 600;
  transition: all 0.2s ease;
}

.header-actions :deep(.el-button--primary:hover:not(:disabled)) {
  background: linear-gradient(145deg, #3498db 0%, #2980b9 100%);
  transform: translateY(-1px);
  box-shadow: 0 4px 12px rgba(52, 152, 219, 0.4);
}

.header-actions :deep(.el-button--primary:active:not(:disabled)) {
  transform: translateY(0);
  box-shadow: 0 2px 6px rgba(52, 152, 219, 0.3);
}

.header-actions :deep(.el-button--primary.is-loading) {
  background: linear-gradient(145deg, #34495e 0%, #2c3e50 100%);
  opacity: 0.6;
}

/* 清理旧版本按钮（红色警告） */
.header-actions :deep(.el-button--danger) {
  background: linear-gradient(145deg, #34495e 0%, #2c3e50 100%);
  border: 1px solid #e74c3c;
  color: #ecf0f1;
  font-weight: 600;
  transition: all 0.2s ease;
}

.header-actions :deep(.el-button--danger:hover:not(:disabled)) {
  background: linear-gradient(145deg, #e74c3c 0%, #c0392b 100%);
  border-color: #e74c3c;
  transform: translateY(-1px);
  box-shadow: 0 4px 12px rgba(231, 76, 60, 0.4);
}

.header-actions :deep(.el-button--danger:active:not(:disabled)) {
  transform: translateY(0);
  box-shadow: 0 2px 6px rgba(231, 76, 60, 0.3);
}

.header-actions :deep(.el-button--danger.is-loading) {
  background: linear-gradient(145deg, #34495e 0%, #2c3e50 100%);
  opacity: 0.6;
}

/* 表格操作按钮 - 切换版本（统一蓝色） */
:deep(.el-table .el-button--primary) {
  background: linear-gradient(145deg, #34495e 0%, #2c3e50 100%);
  border: 1px solid #3498db;
  color: #ecf0f1;
  font-weight: 600;
  transition: all 0.2s ease;
}

:deep(.el-table .el-button--primary:hover:not(:disabled)) {
  background: linear-gradient(145deg, #3498db 0%, #2980b9 100%);
  transform: translateY(-1px);
  box-shadow: 0 4px 12px rgba(52, 152, 219, 0.4);
}

:deep(.el-table .el-button--primary:active:not(:disabled)) {
  transform: translateY(0);
  box-shadow: 0 2px 6px rgba(52, 152, 219, 0.3);
}

/* 表格操作按钮 - 删除（红色警告） */
:deep(.el-table .el-button--danger) {
  background: linear-gradient(145deg, #34495e 0%, #2c3e50 100%);
  border: 1px solid #e74c3c;
  color: #ecf0f1;
  font-weight: 600;
  transition: all 0.2s ease;
}

:deep(.el-table .el-button--danger:hover:not(:disabled)) {
  background: linear-gradient(145deg, #e74c3c 0%, #c0392b 100%);
  border-color: #e74c3c;
  transform: translateY(-1px);
  box-shadow: 0 4px 12px rgba(231, 76, 60, 0.4);
}

:deep(.el-table .el-button--danger:active:not(:disabled)) {
  transform: translateY(0);
  box-shadow: 0 2px 6px rgba(231, 76, 60, 0.3);
}

/* 统一按钮尺寸 - 头部和表格 */
.header-actions :deep(.el-button),
:deep(.el-table .el-button) {
  height: 40px !important;
  font-size: 14px !important;
}

/* 表格按钮的 loading 状态 */
:deep(.el-table .el-button.is-loading) {
  background: linear-gradient(145deg, #34495e 0%, #2c3e50 100%);
  opacity: 0.6;
}

/* Plain 按钮覆盖 Element Plus 默认样式 */
:deep(.el-button.is-plain) {
  background: linear-gradient(145deg, #34495e 0%, #2c3e50 100%) !important;
}

:deep(.el-button--danger.is-plain) {
  border: 1px solid #e74c3c !important;
}

/* 统一工业风格 - 移动端卡片按钮 */
.card-actions-mobile :deep(.el-button) {
  height: 44px !important;  /* 移动端稍大 */
  font-size: 14px !important;
  font-weight: 600;
}

.card-actions-mobile :deep(.el-button--primary) {
  background: linear-gradient(145deg, #34495e 0%, #2c3e50 100%);
  border: 1px solid #3498db;
  color: #ecf0f1;
  transition: all 0.2s ease;
}

.card-actions-mobile :deep(.el-button--primary:hover:not(:disabled)) {
  background: linear-gradient(145deg, #3498db 0%, #2980b9 100%);
  transform: translateY(-1px);
  box-shadow: 0 4px 12px rgba(52, 152, 219, 0.4);
}

.card-actions-mobile :deep(.el-button--primary:active:not(:disabled)) {
  transform: translateY(0);
  box-shadow: 0 2px 6px rgba(52, 152, 219, 0.3);
}

.card-actions-mobile :deep(.el-button--danger) {
  background: linear-gradient(145deg, #34495e 0%, #2c3e50 100%);
  border: 1px solid #e74c3c;
  color: #ecf0f1;
  transition: all 0.2s ease;
}

.card-actions-mobile :deep(.el-button--danger:hover:not(:disabled)) {
  background: linear-gradient(145deg, #e74c3c 0%, #c0392b 100%);
  border-color: #e74c3c;
  transform: translateY(-1px);
  box-shadow: 0 4px 12px rgba(231, 76, 60, 0.4);
}

.card-actions-mobile :deep(.el-button--danger:active:not(:disabled)) {
  transform: translateY(0);
  box-shadow: 0 2px 6px rgba(231, 76, 60, 0.3);
}

/* 统一工业风格 - 对话框按钮（取消/分析/确认清理） */
:deep(.el-dialog__footer .el-button) {
  height: 40px !important;
  font-size: 14px !important;
  font-weight: 600;
  background: linear-gradient(145deg, #34495e 0%, #2c3e50 100%);
  border: 1px solid #3498db;
  color: #ecf0f1;
  transition: all 0.2s ease;
}

:deep(.el-dialog__footer .el-button:hover:not(:disabled)) {
  background: linear-gradient(145deg, #3498db 0%, #2980b9 100%);
  transform: translateY(-1px);
  box-shadow: 0 4px 12px rgba(52, 152, 219, 0.4);
}

:deep(.el-dialog__footer .el-button:active:not(:disabled)) {
  transform: translateY(0);
  box-shadow: 0 2px 6px rgba(52, 152, 219, 0.3);
}

:deep(.el-dialog__footer .el-button--primary) {
  background: linear-gradient(145deg, #34495e 0%, #2c3e50 100%);
  border: 1px solid #3498db;
  color: #ecf0f1;
}

:deep(.el-dialog__footer .el-button--primary:hover:not(:disabled)) {
  background: linear-gradient(145deg, #3498db 0%, #2980b9 100%);
  border-color: #3498db;
}

:deep(.el-dialog__footer .el-button--primary.is-loading) {
  background: linear-gradient(145deg, #34495e 0%, #2c3e50 100%);
  opacity: 0.6;
}

:deep(.el-dialog__footer .el-button--danger) {
  background: linear-gradient(145deg, #34495e 0%, #2c3e50 100%);
  border: 1px solid #e74c3c;
  color: #ecf0f1;
}

:deep(.el-dialog__footer .el-button--danger:hover:not(:disabled)) {
  background: linear-gradient(145deg, #e74c3c 0%, #c0392b 100%);
  border-color: #e74c3c;
  box-shadow: 0 4px 12px rgba(231, 76, 60, 0.4);
}

:deep(.el-dialog__footer .el-button--danger.is-loading) {
  background: linear-gradient(145deg, #34495e 0%, #2c3e50 100%);
  opacity: 0.6;
}
</style>

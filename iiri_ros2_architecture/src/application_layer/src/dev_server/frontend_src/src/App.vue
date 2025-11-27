<template>
  <div id="app">
    <el-container class="app-container" :class="{ 'mobile-layout': isMobile }">
      <!-- 顶部导航栏 -->
      <el-header class="app-header">
        <div class="header-content">
          <div class="header-left">
            <el-button
              v-if="isMobile"
              class="menu-toggle"
              @click="toggleSidebar"
              text
            >
              <el-icon :size="24">
                <Close v-if="sidebarVisible" />
                <Menu v-else />
              </el-icon>
            </el-button>
            <div class="brand">
              <div class="brand-logo">IIRI</div>
              <h1 class="app-title">机器人控制平台</h1>
            </div>
          </div>
          <nav v-if="!isMobile" class="header-nav">
            <a
              class="nav-item"
              :class="{ active: $route.path === '/' }"
              @click="$router.push('/')"
            >
              <el-icon class="nav-icon"><Monitor /></el-icon>
              <span class="nav-label">控制台</span>
            </a>
            <a
              class="nav-item"
              :class="{ active: $route.path === '/ota' }"
              @click="$router.push('/ota')"
            >
              <el-icon class="nav-icon"><Upload /></el-icon>
              <span class="nav-label">系统更新</span>
            </a>
            <a
              class="nav-item"
              :class="{ active: $route.path === '/version' }"
              @click="$router.push('/version')"
            >
              <el-icon class="nav-icon"><InfoFilled /></el-icon>
              <span class="nav-label">版本信息</span>
            </a>
          </nav>
          <div class="header-right">
            <div class="connection-indicator" :class="connectionStatus.type">
              <span class="status-dot"></span>
              <span class="status-text">{{ connectionStatus.text }}</span>
            </div>
          </div>
        </div>
      </el-header>

      <el-container class="main-container">
        <!-- 侧边栏 (只显示连接设置) -->
        <el-aside
          v-if="$route.path === '/'"
          :width="sidebarWidth"
          class="app-sidebar"
          :class="{ 'mobile-sidebar': isMobile, 'sidebar-visible': sidebarVisible || !isMobile }"
        >
          <div class="sidebar-content">
            <ConnectionPanel />
          </div>
        </el-aside>

        <!-- 遮罩层（移动端侧边栏打开时） -->
        <div
          v-if="isMobile && sidebarVisible && $route.path === '/'"
          class="sidebar-overlay"
          @click="toggleSidebar"
        ></div>

        <!-- 主内容区 -->
        <el-main class="app-main" :class="{ 'full-width': $route.path !== '/' }">
          <!-- 主页内容 -->
          <div v-if="$route.path === '/'" class="home-content">
            <!-- PC端布局：左侧（视频+系统信息）+ 右侧（控制面板） -->
            <div v-if="!isMobile" class="main-content-wrapper">
              <!-- 左侧：视频和系统信息 -->
              <div class="left-content">
                <VideoPlayer />
                <div class="info-container">
                  <SystemInfo />
                </div>
              </div>

              <!-- 右侧浮动控制面板 -->
              <div class="floating-control-panel">
                <ControlPanel />
              </div>
            </div>

            <!-- 移动端布局：视频 → 控制面板 → 系统信息 -->
            <div v-if="isMobile" class="mobile-content-wrapper">
              <VideoPlayer />

              <div class="mobile-panel">
                <ControlPanel />
              </div>

              <div class="info-container">
                <SystemInfo />
              </div>
            </div>
          </div>

          <!-- 路由页面 -->
          <router-view v-else />
        </el-main>
      </el-container>

      <!-- 底部导航栏（仅移动端显示） -->
      <el-footer v-if="isMobile" class="bottom-nav" height="60px">
        <div class="bottom-nav-content">
          <a
            class="bottom-nav-item"
            :class="{ active: $route.path === '/' }"
            @click="$router.push('/')"
          >
            <el-icon class="bottom-nav-icon"><Monitor /></el-icon>
            <span class="bottom-nav-label">控制台</span>
          </a>
          <a
            class="bottom-nav-item"
            :class="{ active: $route.path === '/ota' }"
            @click="$router.push('/ota')"
          >
            <el-icon class="bottom-nav-icon"><Upload /></el-icon>
            <span class="bottom-nav-label">系统更新</span>
          </a>
          <a
            class="bottom-nav-item"
            :class="{ active: $route.path === '/version' }"
            @click="$router.push('/version')"
          >
            <el-icon class="bottom-nav-icon"><InfoFilled /></el-icon>
            <span class="bottom-nav-label">版本信息</span>
          </a>
        </div>
      </el-footer>
    </el-container>
  </div>
</template>

<script setup lang="ts">
import { computed, ref, onMounted, onUnmounted } from 'vue'
import { Monitor, Upload, InfoFilled, Menu, Close } from '@element-plus/icons-vue'
import { useWebSocketStore } from './stores/websocket'
import ConnectionPanel from './components/ConnectionPanel.vue'
import ControlPanel from './components/ControlPanel.vue'
import VideoPlayer from './components/VideoPlayer.vue'
import SystemInfo from './components/SystemInfo.vue'

const wsStore = useWebSocketStore()

// 移动端检测
const isMobile = ref(false)
const sidebarVisible = ref(false)

// 检查是否为移动端
const checkMobile = () => {
  isMobile.value = window.innerWidth <= 768
  // 在PC端始终显示侧边栏，移动端默认隐藏
  if (!isMobile.value) {
    sidebarVisible.value = true
  }
}

// 切换侧边栏
const toggleSidebar = () => {
  sidebarVisible.value = !sidebarVisible.value
}

// 侧边栏宽度
const sidebarWidth = computed(() => {
  if (isMobile.value) {
    return '280px'
  }
  return '300px'
})

// 连接状态
const connectionStatus = computed(() => {
  if (wsStore.isConnected) {
    return { type: 'success', text: '已连接' }
  } else if (wsStore.isConnecting) {
    return { type: 'warning', text: '连接中...' }
  } else {
    return { type: 'danger', text: '未连接' }
  }
})

onMounted(() => {
  checkMobile()
  window.addEventListener('resize', checkMobile)
})

onUnmounted(() => {
  window.removeEventListener('resize', checkMobile)
})
</script>

<style scoped>
/* ===== 基础布局 ===== */
.app-container {
  height: 100vh;
  width: 100vw;
  position: relative;
}

.main-container {
  position: relative;
  height: calc(100vh - 60px);
}

/* ===== 顶部导航栏 ===== */
.app-header {
  background: #1e2936;
  color: #ecf0f1;
  display: flex;
  align-items: center;
  height: 60px;
  padding: 0 30px;
  box-shadow: 0 2px 8px rgba(0, 0, 0, 0.3);
  z-index: 1000;
  position: relative;
  border-bottom: 2px solid #34495e;
}

.header-content {
  width: 100%;
  display: flex;
  justify-content: space-between;
  align-items: center;
  position: relative;
  z-index: 1;
}

.header-left {
  display: flex;
  align-items: center;
  gap: 15px;
}

.brand {
  display: flex;
  align-items: center;
  gap: 15px;
}

.brand-logo {
  font-size: 24px;
  font-weight: 800;
  font-family: 'Arial Black', sans-serif;
  color: #3498db;
  background: linear-gradient(135deg, #3498db 0%, #2980b9 100%);
  -webkit-background-clip: text;
  -webkit-text-fill-color: transparent;
  background-clip: text;
  letter-spacing: 2px;
  border: 2px solid #3498db;
  padding: 4px 12px;
  border-radius: 4px;
}

.app-title {
  margin: 0;
  font-size: 18px;
  font-weight: 500;
  white-space: nowrap;
  letter-spacing: 1px;
  color: #ecf0f1;
  font-family: 'Microsoft YaHei', sans-serif;
}

.header-nav {
  display: flex;
  align-items: center;
  gap: 8px;
}

.nav-item {
  display: flex;
  align-items: center;
  gap: 8px;
  padding: 8px 16px;
  border-radius: 4px;
  color: #95a5a6;
  font-size: 14px;
  font-weight: 500;
  cursor: pointer;
  transition: all 0.2s ease;
  background: transparent;
  border: 1px solid transparent;
  position: relative;
}

.nav-item:hover {
  background: #2c3e50;
  color: #ecf0f1;
  border-color: #34495e;
}

.nav-item.active {
  background: #34495e;
  color: #3498db;
  border-color: #3498db;
}

.nav-item.active::before {
  content: '';
  position: absolute;
  bottom: -2px;
  left: 0;
  right: 0;
  height: 2px;
  background: #3498db;
}

.nav-icon {
  font-size: 16px;
}

.nav-label {
  white-space: nowrap;
}

.header-right {
  display: flex;
  align-items: center;
}

.connection-indicator {
  display: flex;
  align-items: center;
  gap: 8px;
  padding: 6px 14px;
  border-radius: 4px;
  background: #2c3e50;
  border: 1px solid #34495e;
}

.status-dot {
  width: 10px;
  height: 10px;
  border-radius: 50%;
  animation: pulse 2s ease-in-out infinite;
}

@keyframes pulse {
  0%, 100% {
    opacity: 1;
  }
  50% {
    opacity: 0.5;
  }
}

.connection-indicator.success .status-dot {
  background: #4ade80;
  box-shadow: 0 0 10px rgba(74, 222, 128, 0.5);
}

.connection-indicator.warning .status-dot {
  background: #fbbf24;
  box-shadow: 0 0 10px rgba(251, 191, 36, 0.5);
}

.connection-indicator.danger .status-dot {
  background: #f87171;
  box-shadow: 0 0 10px rgba(248, 113, 113, 0.5);
}

.status-text {
  font-size: 14px;
  font-weight: 500;
  color: white;
}

.menu-toggle {
  color: #ecf0f1 !important;
  font-size: 24px;
  padding: 8px;
  background: transparent !important;
}

.menu-toggle:hover {
  color: #3498db !important;
  background: rgba(52, 152, 219, 0.1) !important;
}

.menu-toggle:active {
  color: #3498db !important;
  background: rgba(52, 152, 219, 0.2) !important;
}

.menu-toggle:focus {
  color: #ecf0f1 !important;
  background: transparent !important;
}

/* ===== 侧边栏 ===== */
.app-sidebar {
  background-color: #2c3e50;
  overflow-y: auto;
  transition: transform 0.3s ease;
  position: relative;
  z-index: 999;
  border-right: 1px solid #34495e;
}

.sidebar-content {
  padding: 20px;
}

/* 移动端侧边栏 */
.mobile-sidebar {
  position: fixed;
  top: 60px;
  left: 0;
  height: calc(100vh - 60px);
  transform: translateX(-100%);
  box-shadow: 2px 0 8px rgba(0, 0, 0, 0.5);
  z-index: 999;
}

.mobile-sidebar.sidebar-visible {
  transform: translateX(0);
}

/* 侧边栏遮罩层 */
.sidebar-overlay {
  position: fixed;
  top: 60px;
  left: 0;
  right: 0;
  bottom: 0;
  background-color: rgba(0, 0, 0, 0.6);
  z-index: 998;
  animation: fadeIn 0.3s;
}

@keyframes fadeIn {
  from {
    opacity: 0;
  }
  to {
    opacity: 1;
  }
}


/* ===== 主内容区 ===== */
.app-main {
  padding: 20px;
  background-color: #ecf0f1;
  display: flex;
  flex-direction: column;
  gap: 20px;
  overflow-y: auto;
}

.app-main.full-width {
  width: 100%;
}

.home-content {
  display: flex;
  flex-direction: column;
  gap: 20px;
  width: 100%;
  height: 100%;
}

/* 主内容包装器 - 水平布局 */
.main-content-wrapper {
  display: flex;
  flex-direction: row;
  gap: 20px;
  width: 100%;
  flex: 1;
}

/* 左侧内容区（视频+系统信息） */
.left-content {
  flex: 1;
  display: flex;
  flex-direction: column;
  gap: 20px;
  min-width: 0; /* 防止flex item溢出 */
}

.info-container {
  min-height: 200px;
  flex-shrink: 0;
  position: relative;
  z-index: 2;
}

/* 右侧浮动控制面板 */
.floating-control-panel {
  width: 320px;
  flex-shrink: 0;
  background: #2c3e50;
  border-radius: 8px;
  box-shadow: 0 4px 12px rgba(0, 0, 0, 0.3);
  overflow-y: auto;
  max-height: calc(100vh - 100px);
  position: sticky;
  top: 0;
}

/* ===== 移动端布局 ===== */
.mobile-content-wrapper {
  display: flex;
  flex-direction: column;
  gap: 16px;
  width: 100%;
}

.mobile-panel {
  background-color: #2c3e50;
  border-radius: 8px;
  padding: 16px;
}

/* ===== 底部导航栏（移动端） ===== */
.bottom-nav {
  position: fixed;
  bottom: 0;
  left: 0;
  right: 0;
  background: #1e2936;
  border-top: 2px solid #34495e;
  box-shadow: 0 -2px 8px rgba(0, 0, 0, 0.3);
  z-index: 1000;
  padding: 0;
}

.bottom-nav-content {
  height: 100%;
  display: flex;
  justify-content: space-around;
  align-items: center;
  padding: 0 8px;
}

.bottom-nav-item {
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  gap: 4px;
  padding: 8px 16px;
  border-radius: 8px;
  color: #95a5a6;
  font-size: 12px;
  cursor: pointer;
  transition: all 0.2s ease;
  min-height: 44px;
  min-width: 44px;
  flex: 1;
}

.bottom-nav-item:active {
  background: #2c3e50;
  transform: scale(0.95);
}

.bottom-nav-item.active {
  color: #3498db;
  background: #34495e;
}

.bottom-nav-icon {
  font-size: 20px;
}

.bottom-nav-label {
  font-size: 11px;
  font-weight: 500;
}

/* ===== 移动端适配 (768px 以下) ===== */
@media screen and (max-width: 768px) {
  .app-header {
    padding: 0 12px;
    height: 60px;
  }

  .brand-logo {
    font-size: 18px;
    padding: 3px 8px;
  }

  .app-title {
    font-size: 13px;
  }

  .nav-item {
    padding: 6px 10px;
    font-size: 13px;
  }

  .nav-icon {
    font-size: 16px;
  }

  .nav-label {
    display: none;
  }

  .connection-indicator {
    padding: 5px 10px;
  }

  .status-text {
    font-size: 12px;
  }

  .main-container {
    height: calc(100vh - 120px); /* 减去顶部导航 60px 和底部导航 60px */
  }

  .app-main {
    padding: 12px;
    padding-bottom: 72px; /* 为底部导航留出空间 */
    gap: 12px;
  }

  .info-container {
    min-height: 150px;
  }

  /* 移动端布局调整 */
  .mobile-layout .app-main {
    width: 100%;
  }
}

/* ===== 小屏手机适配 (480px 以下) ===== */
@media screen and (max-width: 480px) {
  .app-title {
    font-size: 14px;
  }

  .app-main {
    padding: 8px;
    gap: 8px;
  }

  .info-container {
    min-height: 120px;
  }

  .sidebar-content {
    padding: 12px;
  }
}

/* ===== 平板适配 (768px - 1024px) ===== */
@media screen and (min-width: 769px) and (max-width: 1024px) {
  .app-sidebar {
    width: 280px !important;
  }

  .app-title {
    font-size: 20px;
  }
}

/* ===== 大屏适配 (1920px 以上) ===== */
@media screen and (min-width: 1920px) {
  .app-title {
    font-size: 28px;
  }

  .info-container {
    min-height: 250px;
  }
}

/* ===== 横屏手机适配 ===== */
@media screen and (max-height: 500px) and (orientation: landscape) {
  .info-container {
    min-height: 100px;
  }

  .app-main {
    gap: 8px;
  }
}
</style>

<style>
html,
body {
  margin: 0;
  padding: 0;
  font-family: 'Helvetica Neue', Helvetica, 'PingFang SC', 'Hiragino Sans GB', 'Microsoft YaHei', '微软雅黑', Arial, sans-serif;
  /* 移动端优化 */
  -webkit-font-smoothing: antialiased;
  -moz-osx-font-smoothing: grayscale;
  /* 禁用双击缩放 */
  touch-action: manipulation;
  /* 优化触摸滚动 */
  -webkit-overflow-scrolling: touch;
}

* {
  box-sizing: border-box;
}

/* 移动端触摸优化 */
@media (hover: none) and (pointer: coarse) {
  /* 增大触摸目标大小 */
  button,
  a,
  input,
  .el-button {
    min-height: 44px;
    min-width: 44px;
  }
}

/* 禁用移动端长按选择 */
img,
video {
  -webkit-user-select: none;
  user-select: none;
  -webkit-touch-callout: none;
}

/* 优化滚动条样式 */
::-webkit-scrollbar {
  width: 8px;
  height: 8px;
}

::-webkit-scrollbar-track {
  background: #f1f1f1;
  border-radius: 4px;
}

::-webkit-scrollbar-thumb {
  background: #888;
  border-radius: 4px;
}

::-webkit-scrollbar-thumb:hover {
  background: #555;
}

/* 移动端隐藏滚动条 */
@media screen and (max-width: 768px) {
  ::-webkit-scrollbar {
    width: 4px;
    height: 4px;
  }
}
</style>
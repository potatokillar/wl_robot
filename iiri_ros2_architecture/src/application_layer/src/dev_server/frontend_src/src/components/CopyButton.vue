<template>
  <el-tooltip
    :content="tooltipText"
    placement="top"
    :show-after="300"
  >
    <el-button
      :icon="copied ? Check : DocumentCopy"
      :size="size"
      text
      circle
      class="copy-button"
      @click="handleCopy"
      :class="{ 'copy-button--copied': copied }"
      :aria-label="copied ? 'Copied!' : 'Copy to clipboard'"
    />
  </el-tooltip>
</template>

<script setup lang="ts">
import { ref, computed } from 'vue'
import { DocumentCopy, Check } from '@element-plus/icons-vue'
import { ElMessage } from 'element-plus'

interface Props {
  value: string
  size?: 'small' | 'default' | 'large'
  successMessage?: string
}

const props = withDefaults(defineProps<Props>(), {
  size: 'default',
  successMessage: '已复制到剪贴板'
})

const copied = ref(false)
let resetTimer: NodeJS.Timeout | null = null

const tooltipText = computed(() =>
  copied.value ? '已复制！' : '点击复制'
)

const handleCopy = async () => {
  try {
    await navigator.clipboard.writeText(props.value)
    copied.value = true
    ElMessage.success(props.successMessage)

    if (resetTimer) clearTimeout(resetTimer)
    resetTimer = setTimeout(() => {
      copied.value = false
    }, 2000)
  } catch (err) {
    ElMessage.error('复制失败')
    console.error('Copy failed:', err)
  }
}
</script>

<style scoped>
.copy-button {
  opacity: 0.6;
  transition: all 0.2s cubic-bezier(0.4, 0, 0.2, 1);
  flex-shrink: 0;
}

.copy-button:hover {
  opacity: 1;
  color: var(--el-color-info);
  transform: scale(1.1);
}

.copy-button:active {
  transform: scale(0.95);
}

.copy-button--copied {
  color: var(--el-color-success);
  opacity: 1;
}
</style>

/**
 * 节流函数 - 限制函数调用频率
 * @param func 需要节流的函数
 * @param delay 延迟时间（毫秒）
 * @returns 节流后的函数
 */
export function throttle<T extends (...args: any[]) => any>(
  func: T,
  delay: number
): (...args: Parameters<T>) => void {
  let lastCall = 0
  let timeoutId: NodeJS.Timeout | null = null

  return function (this: any, ...args: Parameters<T>) {
    const now = Date.now()
    const timeSinceLastCall = now - lastCall

    // 如果距离上次调用已经超过delay，立即执行
    if (timeSinceLastCall >= delay) {
      lastCall = now
      func.apply(this, args)
    } else {
      // 否则，清除之前的定时器，设置新的定时器
      if (timeoutId) {
        clearTimeout(timeoutId)
      }
      timeoutId = setTimeout(() => {
        lastCall = Date.now()
        func.apply(this, args)
      }, delay - timeSinceLastCall)
    }
  }
}

/**
 * 防抖函数 - 延迟执行，多次调用只执行最后一次
 * @param func 需要防抖的函数
 * @param delay 延迟时间（毫秒）
 * @returns 防抖后的函数
 */
export function debounce<T extends (...args: any[]) => any>(
  func: T,
  delay: number
): (...args: Parameters<T>) => void {
  let timeoutId: NodeJS.Timeout | null = null

  return function (this: any, ...args: Parameters<T>) {
    if (timeoutId) {
      clearTimeout(timeoutId)
    }
    timeoutId = setTimeout(() => {
      func.apply(this, args)
    }, delay)
  }
}

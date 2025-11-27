import { defineStore } from 'pinia'
import { ref } from 'vue'

export const useAudioStore = defineStore('audio', () => {
    // 音频状态
    const isMuted = ref(false)
    const isTalking = ref(false)
    const volume = ref(50)
    const isRecording = ref(false)

    // Web Audio API 相关
    let audioContext: AudioContext | null = null
    let mediaRecorder: MediaRecorder | null = null
    let audioStream: MediaStream | null = null

    /**
     * 初始化音频系统
     */
    async function initAudio() {
        try {
            // 初始化 AudioContext
            audioContext = new (window.AudioContext || (window as any).webkitAudioContext)()

            // 请求麦克风权限
            audioStream = await navigator.mediaDevices.getUserMedia({
                audio: {
                    echoCancellation: true,
                    noiseSuppression: true,
                    sampleRate: 16000,
                    channelCount: 1
                }
            })

            console.log('音频系统初始化成功')
            return true
        } catch (error) {
            console.error('音频系统初始化失败:', error)
            return false
        }
    }

    /**
     * 开始录音
     */
    async function startRecording(onDataAvailable: (data: ArrayBuffer) => void) {
        if (!audioStream) {
            const success = await initAudio()
            if (!success) return false
        }

        try {
            mediaRecorder = new MediaRecorder(audioStream!, {
                mimeType: 'audio/webm;codecs=opus'
            })

            mediaRecorder.ondataavailable = (event) => {
                if (event.data.size > 0) {
                    event.data.arrayBuffer().then(buffer => {
                        onDataAvailable(buffer)
                    })
                }
            }

            mediaRecorder.start(100) // 每100ms收集一次数据
            isRecording.value = true
            console.log('开始录音')
            return true
        } catch (error) {
            console.error('开始录音失败:', error)
            return false
        }
    }

    /**
     * 停止录音
     */
    function stopRecording() {
        if (mediaRecorder && mediaRecorder.state === 'recording') {
            mediaRecorder.stop()
            isRecording.value = false
            console.log('停止录音')
        }
    }

    /**
     * 播放音频数据
     */
    async function playAudioData(audioData: ArrayBuffer) {
        if (!audioContext) {
            await initAudio()
        }

        try {
            const audioBuffer = await audioContext!.decodeAudioData(audioData.slice(0))
            const source = audioContext!.createBufferSource()

            const gainNode = audioContext!.createGain()
            gainNode.gain.value = isMuted.value ? 0 : volume.value / 100

            source.buffer = audioBuffer
            source.connect(gainNode)
            gainNode.connect(audioContext!.destination)

            source.start()
        } catch (error) {
            console.error('播放音频失败:', error)
        }
    }

    /**
     * 切换静音状态
     */
    function toggleMute() {
        isMuted.value = !isMuted.value
    }

    /**
     * 设置音量
     */
    function setVolume(newVolume: number) {
        volume.value = Math.max(0, Math.min(100, newVolume))
    }

    /**
     * 设置对讲状态
     */
    function setTalking(talking: boolean) {
        isTalking.value = talking
    }

    /**
     * 清理音频资源
     */
    function cleanup() {
        stopRecording()

        if (audioStream) {
            audioStream.getTracks().forEach(track => track.stop())
            audioStream = null
        }

        if (audioContext) {
            audioContext.close()
            audioContext = null
        }
    }

    return {
        // 状态
        isMuted,
        isTalking,
        volume,
        isRecording,

        // 方法
        initAudio,
        startRecording,
        stopRecording,
        playAudioData,
        toggleMute,
        setVolume,
        setTalking,
        cleanup
    }
})
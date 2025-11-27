
#include "eventMsg.hpp"

#include <map>

std::map<QrEventType, std::function<void(QrEventType)>> qrEventMap;

void SetQrEvent(QrEventType type)
{
    if (qrEventMap.count(type) > 0) {
        qrEventMap.at(type)(type);
    }
}

/**
 * @description: 设置事件回调函数，该函数不能阻塞，一个事件只支持一个
 * @param type
 * @param func
 * @return {}
 */
void SetQrEventCallback(QrEventType type, std::function<void(QrEventType)> func) { qrEventMap[type] = func; }

#if 0
/**
 * @description: 设置事件回调函数，该函数需要较长的运行时间
 * @param type
 * @param func
 * @return {}
 */
void SetQrEventCallbackBlock(QrEventType type, std::function<void(QrEventType)> func)
{
}
#endif
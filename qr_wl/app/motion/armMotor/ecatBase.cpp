
#include "ecatBase.hpp"

namespace ethercat
{

/**
 * @description: pdo映射参数获取
 * @param index 对象字典索引
 * @param subIndex 对象字典子索引
 * @param len 对象字典数据长度(1/2/4字节)
 * @return {}
 */
u32 GetPdoMapData(u16 index, u8 subIndex, u8 len)
{
    u8 data = 0;
    switch (len) {
        case 1:
            data = 0x08;
            break;
        case 2:
            data = 0x10;
            break;
        case 4:
            data = 0x20;
            break;
        default:
            break;
    }
    u32 ret = (index << 16) | (subIndex << 8) | data;
    return ret;
}
}  // namespace ethercat
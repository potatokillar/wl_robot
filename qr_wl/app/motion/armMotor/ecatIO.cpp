#include "ecatIO.hpp"

#include <soem/ethercat.h>

#include <iostream>

#include "baseline.hpp"

// 发送给从机的默认PDO
struct RPDO1_IO
{
    u16 ioOut;
};

// 从机发出的默认PDO
struct TPDO1_IO
{
    u16 ioIn;
};
#pragma pack()
// pdo 通讯参数
inline constexpr u16 RPDO1_comm = 0x1400;
inline constexpr u16 RPDO2_comm = RPDO1_comm + 1;
inline constexpr u16 RPDO3_comm = RPDO1_comm + 2;
inline constexpr u16 RPDO4_comm = RPDO1_comm + 3;

inline constexpr u16 TPDO1_comm = 0x1800;
inline constexpr u16 TPDO2_comm = TPDO1_comm + 1;
inline constexpr u16 TPDO3_comm = TPDO1_comm + 2;
inline constexpr u16 TPDO4_comm = TPDO1_comm + 3;

// pdo 映射参数
inline constexpr u16 RPDO1_map = 0x1600;
inline constexpr u16 RPDO2_map = RPDO1_map + 1;
inline constexpr u16 RPDO3_map = RPDO1_map + 2;
inline constexpr u16 RPDO4_map = RPDO1_map + 3;

inline constexpr u16 TPDO1_map = 0x1A00;
inline constexpr u16 TPDO2_map = TPDO1_map + 1;
inline constexpr u16 TPDO3_map = TPDO1_map + 2;
inline constexpr u16 TPDO4_map = TPDO1_map + 3;
EcatIO::EcatIO(int slc) : slc_(slc) {}

u32 EcatIO::GetPdoMapData(u16 index, u8 subIndex, u8 len)
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

/**
 * @description: pre-op下的初始化
 * @return {}
 */
void EcatIO::PreOpInit()
{
    // 设置为freerun
    u16 data16 = 0;
    ec_SDOwrite(slc_, 0x1C32, 0x01, FALSE, 2, &data16, EC_TIMEOUTRXM);

    //  int len16 = 2;
    // ec_SDOread(slc_, 0x1C32, 0x01, FALSE, &len16, &data16, EC_TIMEOUTRXM);
    // LOG_INFO("slave:{}, 1C32-01: {}", slc_, data16);
}

void EcatIO::SetIoOut(u16 set)
{
    auto output = (RPDO1_IO*)ec_slave[slc_].outputs;
    output->ioOut = set;
    // printf("ioO 0x%02X\n", output->ioOut);
}

/**
 * @description: 目前暂时使用SDO
 * @param idx
 * @param set
 * @return {}
 */
void EcatIO::SetIoOut(int idx, bool set)
{
    RPDO1_IO* output = (RPDO1_IO*)ec_slave[slc_].outputs;
    if (set) {
        output->ioOut |= 1 << idx;
    } else {
        output->ioOut &= ~(1 << idx);
    }

    // printf("ioO 0x%02X\n", output->ioOut);
}

bool EcatIO::GetIoIn(int idx)
{
    u32 ret = GetIoIn();
    return ret & (1 << idx);
}

u16 EcatIO::GetIoIn()
{
    TPDO1_IO* input = (TPDO1_IO*)ec_slave[slc_].inputs;
    // printf("ioI 0x%02X\n", input->ioIn);
    return input->ioIn;
}
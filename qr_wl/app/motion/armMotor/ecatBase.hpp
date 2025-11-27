
#pragma once

#include "baseline.hpp"

namespace ethercat
{

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

u32 GetPdoMapData(u16 index, u8 subIndex, u8 len);
};
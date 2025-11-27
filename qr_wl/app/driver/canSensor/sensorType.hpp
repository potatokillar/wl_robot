
#pragma once
#include "baseline.hpp"

template <typename T>
class TimestampedData
{
public:
    T data;              // 数据成员
    uint64_t timestamp;  // 时间戳成员
};

// 温度信息
struct TemperatureInfo
{
    TimestampedData<s16> batteryTemperature1{0, 0};  // 电源板获取的外购电池的温度，
    TimestampedData<s16> batteryTemperature2{0, 0};
    TimestampedData<s16> batteryTemperature3{0, 0};
    TimestampedData<s16> batteryTemperature4{0, 0};

    TimestampedData<s16> powerPcbTemperature{0, 0};  // 电源板的温度

    TimestampedData<s16> bmsTemperature1{0, 0};  // bms温度探头1
    TimestampedData<s16> bmsTemperature2{0, 0};  // bms温度探头2
    TimestampedData<s16> bmsTemperature3{0, 0};  // bms温度探头3

    TimestampedData<s16> bmsTemperatureState{0, 0};
};

// 电源信息
struct PowerInfo
{
    std::string name = "";  // 电源模块名字，
    u32 kind = 0;           // 电源种类，
    u32 index = 0;
    bool state = false;
    u32 powerIndexMotor = 0;
    bool powerStateMotor = false;
    u32 powerIndexBMS = 0;
    bool BMSState = false;
};

// 电机电流信息
struct MotorCurrentInfo
{
    TimestampedData<s16> current1{0, 0};
    TimestampedData<s16> current2{0, 0};
    TimestampedData<s16> current3{0, 0};
    TimestampedData<s16> current4{0, 0};
    TimestampedData<s16> current1Max{0, 0};
    TimestampedData<s16> current3Max{0, 0};
};

// 控制板信息
struct BoardInfo
{
    std::string name = "";
    u32 canId = 0xFF;  // 0xFF为默认值
};

// 电池基础信息
struct BasicBatteryInfo
{
    TimestampedData<u16> quantity{100, 0};             // 电量，范围0~100
    TimestampedData<u8> lastPowerOffReason{0, 0};      // bms上一次断电原因
    TimestampedData<u8> previousPowerOffReason{0, 0};  // bms上上一次断电原因
    TimestampedData<u8> priorPowerOffReason{0, 0};     // bms上上上一次断电原因
};

// 电池详细信息
struct DetailedBatteryInfo
{
    TimestampedData<s16> tempCMax{0, 0};
    TimestampedData<double> voltage{0, 0};
    TimestampedData<double> current{0, 0};
    TimestampedData<double> currentMax{0, 0};
    TimestampedData<uint8_t> cellNum = {0, 0};
    TimestampedData<double> cellVoltage[16] = {{0, 0}};
};
enum class PowerOffReason : uint8_t
{
    BmsInitialPowerOn = 0x00,
    NormalButtonShutdown = 0x01,
    LowBatteryVoltageShutdown = 0x02,
    OvercurrentProtectionShutdown = 0x03,
    ShortCircuitProtectionShutdown = 0x04,
    UnderVoltageProtectionShutdown = 0x05,
    OverVoltageProtectionShutdown = 0x06,
    SoftwareInitiatedShutdown = 0x07,
    TotalCurrentAnomaly = 0x08,
    LoadDisconnected = 0x09,
    Invalid = 0xFF
};

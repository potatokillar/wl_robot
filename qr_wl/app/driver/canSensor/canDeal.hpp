
#pragma once
#include <map>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

#include "baseline.hpp"
#include "canProtocol.hpp"
#include "sensorType.hpp"
#include "uart.hpp"

class CanDeal
{
public:
    CanDeal();
    virtual ~CanDeal();

public:
    // 主动查询
    MsgType GetBoardInfo(const MsgType& set);
    MsgType SetMotorPower(const MsgType& in);
    MsgType GetMotorPower(const MsgType& in);
    // 定时上报
    TemperatureInfo GetTemperatureInfo();
    MotorCurrentInfo GetCurrentInfo();
    DetailedBatteryInfo GetDetailedBatteryInfo();
    BasicBatteryInfo GetBasicBatteryInfo();
    // 事件触发上报
    TimestampedData<bool> GetEmergStopInfo();

private:
    void RxCurrentInfo(const SensorCanData& data);
    void RxBoardInfo(const SensorCanData& data);
    void RxSetPowerState(const SensorCanData& data);
    void RxGetPowerInfo(const SensorCanData& data);
    void RxTemperatureInfo(const SensorCanData& data);
    void RxDetailedBatteryInfo(const SensorCanData& data);
    void RxBasicBatteryInfo(const SensorCanData& data);
    void RxEmergStopInfo(const SensorCanData& data);

    std::pair<bool, std::vector<BoardInfo>> boardInfo_;
    std::pair<bool, PowerInfo> motorOrBMSPowerInfo_;
    TemperatureInfo temperatureInfo_;
    MotorCurrentInfo currentInfo_;
    DetailedBatteryInfo detailedBatteryInfo_;
    BasicBatteryInfo basicBatteryInfo_;
    TimestampedData<bool> emergStopInfo_;

    CanProtocol canProto_;  // CanDeal嵌套了CanProtocol类
    mutable std::mutex rxMutex_;

    std::array<std::condition_variable, 256> cv_;  // 每一个数据一个条件变量
};

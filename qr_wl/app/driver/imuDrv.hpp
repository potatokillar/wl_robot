
#pragma once

#include <queue>
#include <vector>

#include "baseline.hpp"
#include "uart.hpp"

struct ImuDataInfo
{
    u64 startTime{0};
    u64 lastReadOk{0};

    std::uint64_t timeIntervalForPacketDifference = 0;
    uint64_t packetCount[3] = {0};
    bool frequencyLog{false};

    std::uint64_t imuTime1 = 0;
    uint64_t packetCount1[3] = {0};

    bool initPrintFlag{false};  // 初始化打印，一次
    bool imuHasError{false};
};

class ImuNode final : public Singleton<ImuNode>
{
public:
    void Init() override;
    void Loop() override;
    void ErrorDetect();

private:
    std::pair<bool, msg::imu_data> ParseData(std::queue<uint8_t>& q);
    ImuDataInfo imuInfo;
    std::unique_ptr<Uart> uart_;
    msg::imu_data imuDataIncomplete_;
    std::queue<uint8_t> uartData_;  // 原始串口数据队列
    bool isOk_{false};
    bool update_{false};
};

inline ImuNode& GetImuNode() { return ImuNode::GetInstance(); }

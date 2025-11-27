
#pragma once

#include <cstdint>
#include <cstring>
#include <deque>
#include <mutex>
#include <vector>

#include "baseline.hpp"
#include "spi.hpp"

struct SPI_TO_CAN_S
{
    union
    {
        uint8_t data[6][8];
        struct
        {
            uint8_t leg_right_abad[8];
            uint8_t leg_right_hip[8];
            uint8_t leg_right_knee[8];
            uint8_t leg_left_abad[8];
            uint8_t leg_left_hip[8];
            uint8_t leg_left_knee[8];
        };
    } payload;
    uint32_t checksum;
    SPI_TO_CAN_S() { memset(this, 0xFF, sizeof(SPI_TO_CAN_S)); }
    void Clear() { memset(this, 0xFF, sizeof(SPI_TO_CAN_S)); }
};

struct MotorInfo
{
    bool timeout;
    u64 lastRecv;
    uint32_t sendCnt{0};  // 发送接收次数
    uint32_t recvCnt{0};
};

struct LegInfo
{
    MotorInfo motor[3];
};

class SpiToCan
{
public:
    SpiToCan(const std::array<std::string, 2> &spiDev, u32 spiSpd);
    SpiToCan(const SpiToCan &) = delete;
    SpiToCan &operator=(const SpiToCan &) = delete;

    void AddMessage(int board, const SPI_TO_CAN_S &msg);
    SPI_TO_CAN_S GetMessage(int board) const;
    void TimeoutCheck();
    void Transfer();
    void ResetTimer();
    void ClearBuf();

    Mat43<bool> GetMotorState() const;

private:
    uint32_t XorChecksum(uint8_t *data, uint32_t len);

private:
    SpiMaster spi_[2];
    bool initOk_{false};

    SPI_TO_CAN_S sendBuf[2];
    SPI_TO_CAN_S recvBuf[2];

    void InitSpi(const std::array<std::string, 2> &spiDev, u32 spiSpd);
    LegInfo leg[4];
    std::map<int, std::string> legNameMap_;
    Mat43<bool> motorSta_;
    bool isCheckErr_{false};
    std::deque<bool> checkErrT_;
};

using MitCanInterface = SpiToCan;

#pragma once

#include <cstdint>
#include <cstring>
#include <deque>
#include <mutex>
#include <vector>

#include "baseline.hpp"
#include "spi.hpp"

// 版本2的spi2can数据结构
struct SPI_TO_CAN_V2
{
    union
    {
        uint8_t data[8][10];
        struct
        {
            // 两路CAN，每路固定4个数据
            uint8_t can0[4][10];
            uint8_t can1[4][10];
        };
    } payload;
    uint32_t checksum;
    SPI_TO_CAN_V2() { memset(this, 0xFF, sizeof(SPI_TO_CAN_V2)); }
    void Clear() { memset(this, 0xFF, sizeof(SPI_TO_CAN_V2)); }
};

// 第二版spi2can接口
class SpiToCanV2
{
public:
    SpiToCanV2(const std::array<std::string, 2> &spiDev, u32 spiSpd);
    SpiToCanV2(const SpiToCanV2 &) = delete;
    SpiToCanV2 &operator=(const SpiToCanV2 &) = delete;

    void AddMessage(int board, const SPI_TO_CAN_V2 &msg);
    SPI_TO_CAN_V2 GetMessage(int board) const;
    void Transfer();
    void ClearBuf();

private:
    uint32_t XorChecksum(uint8_t *data, uint32_t len);
    void ErrorCheck();

private:
    SpiMaster spi_[2];
    bool isOk_{false};

    SPI_TO_CAN_V2 sendBuf[2];
    SPI_TO_CAN_V2 recvBuf[2];

    void InitSpi(const std::array<std::string, 2> &spiDev, u32 spiSpd);
    std::deque<bool> checkErrT_;
};

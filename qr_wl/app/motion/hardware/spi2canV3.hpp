
#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <deque>
#include <memory>
#include <mutex>
#include <vector>

#include "baseline.hpp"
#include "spi.hpp"

struct alignas(1) CanData
{
    u16 id;
    u8 data[8];
    CanData()
    {
        id = 0x8000;
        for (int i = 0; i < 8; i++) {
            data[i] = 0xFF;
        }
    }
};

// 标记数据的位置
struct SpiDataOffset
{
    u8 start{0};
    u8 can0_num{1};
    u8 can1_num{2};
    std::vector<u8> can0_data;
    std::vector<u8> can1_data;
    u8 check;
    u8 end;
};

class SpiToCanV3
{
public:
    SpiToCanV3(const std::array<std::string, 2> &spiDev, u32 spiSpd, const std::array<size_t, 4> &canNum);
    SpiToCanV3(const SpiToCanV3 &) = delete;
    SpiToCanV3 &operator=(const SpiToCanV3 &) = delete;

    bool AddMessage(int board, const std::vector<CanData> &msg1, const std::vector<CanData> &msg2);
    std::vector<CanData> GetMessage(int board, int canIdx) const;
    void Transfer();
    void ClearBuf();
    void ShowFrame(std::array<std::vector<u8>, 2>& buf_, bool ShowSendFrame = false);

private:
    uint32_t XorChecksum(std::vector<u8> &data, uint32_t len);
    void ErrorCheck();

private:
    SpiMaster spi_[2];
    bool isOk_{false};
    std::array<size_t, 4> canNum_;

    std::array<std::vector<u8>, 2> txbuf_;
    std::array<std::vector<u8>, 2> rxbuf_;
    std::array<std::vector<CanData>, 4> rxCan_;

    void InitSpi(const std::array<std::string, 2> &spiDev, u32 spiSpd);
    std::deque<bool> checkErrT_;
};

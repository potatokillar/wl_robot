#include "spi2canV2.hpp"

#include <fcntl.h>  //Needed for SPI port
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>  //Needed for SPI port

#include <chrono>

#include "baseline.hpp"

using namespace ::std;
static constexpr int SPI2CAN_SIZE = sizeof(SPI_TO_CAN_V2);
static constexpr int CHECKSUM_SIZE = SPI2CAN_SIZE - 4;
static unsigned char spi_mode = SPI_MODE_0;
static unsigned char spi_bits_per_word = 8;
static unsigned int spi_speed = 3000000;
static uint8_t lsb = 0;

SpiToCanV2::SpiToCanV2(const std::array<std::string, 2> &spiDev, u32 spiSpd) { InitSpi(spiDev, spiSpd); }

void SpiToCanV2::AddMessage(int board, const SPI_TO_CAN_V2 &msg) { sendBuf[board] = msg; }

SPI_TO_CAN_V2 SpiToCanV2::GetMessage(int board) const { return recvBuf[board]; }

void SpiToCanV2::Transfer()
{
    if (isOk_ == false) return;

    sendBuf[0].checksum = XorChecksum((uint8_t *)&sendBuf[0], CHECKSUM_SIZE);
    sendBuf[1].checksum = XorChecksum((uint8_t *)&sendBuf[1], CHECKSUM_SIZE);

    for (int i = 0; i < 2; i++) {
        spi_[i].WriteAndRead((u8 *)&sendBuf[i], (u8 *)&recvBuf[i], SPI2CAN_SIZE);
    }
    ErrorCheck();
}

uint32_t SpiToCanV2::XorChecksum(uint8_t *data, uint32_t len)
{
    uint32_t *doubleWordData = (uint32_t *)data;
    uint32_t doubleWordlen = len / 4;
    uint32_t ret = 0;
    for (size_t i = 0; i < doubleWordlen; i++) {
        ret = ret ^ doubleWordData[i];
    }

    ret += 0xAA;
    return ret;
}

void SpiToCanV2::InitSpi(const std::array<std::string, 2> &spiDev, u32 spiSpd)
{
    spi_speed = spiSpd;

    for (int i = 0; i < 2; i++) {
        if (spi_[i].Open(spiDev[i]) == false) {
            return;
        }
        if (spi_[i].SetParam(spi_mode, spi_speed, spi_bits_per_word, lsb) == false) {
            return;
        }
    }

    isOk_ = true;
}

void SpiToCanV2::ErrorCheck()
{
    bool chkErr = false;
    for (int i = 0; i < 2; i++) {
        uint32_t expectChecksum = XorChecksum((uint8_t *)&recvBuf[i], CHECKSUM_SIZE);
        if (expectChecksum != recvBuf[i].checksum) {
            LOG_WARN("board {} spi checksum error, actul:{:X}, except:{:X}", i, recvBuf[i].checksum, expectChecksum);
            chkErr = true;
        }
    }

    checkErrT_.push_back(chkErr);
    if (checkErrT_.size() > 10) {
        checkErrT_.pop_front();
    }
    int errSum = 0;
    for (const auto &v : checkErrT_) {
        if (v == true) {
            errSum++;
        }
    }

    if (errSum >= 5) {
        LOG_ERROR("too many spi checksum err, stop!!!");
        isOk_ = false;
    }
}

void SpiToCanV2::ClearBuf()
{
    for (int i = 0; i < 2; i++) {
        recvBuf[i].Clear();
        sendBuf[i].Clear();
    }
}

#include "spi2canV3.hpp"

#include <fcntl.h>  //Needed for SPI port
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>  //Needed for SPI port

#include <chrono>
#include <thread>

#include "baseline.hpp"

using namespace ::std;

static unsigned char spi_mode = SPI_MODE_0;
static unsigned char spi_bits_per_word = 8;
static unsigned int spi_speed = 3000000;
static uint8_t lsb = 0;

#define SPI_FRAME_HEAD_SIZE (3)
#define SPI_FRAME_TAIL_SIZE (1)
#define SPI_FRAME_CHECKSUM_SIZE (4)
#define SPI_FRAME_ALIGN_SIZE (4)

SpiToCanV3::SpiToCanV3(const std::array<std::string, 2> &spiDev, u32 spiSpd, const std::array<size_t, 4> &canNum) : canNum_(canNum)
{
    InitSpi(spiDev, spiSpd);
    int spi0bodylen = SPI_FRAME_HEAD_SIZE + canNum_[0] * 10 + canNum_[1] * 10;
    int spi0alignlen = ((spi0bodylen + SPI_FRAME_ALIGN_SIZE - 1) / SPI_FRAME_ALIGN_SIZE) * SPI_FRAME_ALIGN_SIZE;
    int spi0len =  spi0alignlen + SPI_FRAME_CHECKSUM_SIZE + SPI_FRAME_TAIL_SIZE;
    txbuf_[0].resize(spi0len);
    rxbuf_[0].resize(spi0len);
    txbuf_[0][0] = 0xAA;
    txbuf_[0][1] = canNum_[0];
    txbuf_[0][2] = canNum_[1];
    txbuf_[0][spi0len - 1] = 0xBB;

    int spi1bodylen = SPI_FRAME_HEAD_SIZE + canNum_[2] * 10 + canNum_[3] * 10;
    int spi1alignlen = ((spi1bodylen + SPI_FRAME_ALIGN_SIZE - 1) / SPI_FRAME_ALIGN_SIZE) * SPI_FRAME_ALIGN_SIZE;
    int spi1len = spi1alignlen + SPI_FRAME_CHECKSUM_SIZE + SPI_FRAME_TAIL_SIZE;
    txbuf_[1].resize(spi1len);
    rxbuf_[1].resize(spi1len);
    txbuf_[1][0] = 0xAA;
    txbuf_[1][1] = canNum_[0];
    txbuf_[1][2] = canNum_[1];
    txbuf_[1][spi1len - 1] = 0xBB;
}

bool SpiToCanV3::AddMessage(int board, const std::vector<CanData> &msg1, const std::vector<CanData> &msg2)
{
    if (board == 0) {
        if (msg1.size() != canNum_[0] || msg2.size() != canNum_[1]) {
            return false;
        }
    }
    if (board == 1) {
        if (msg1.size() != canNum_[2] || msg2.size() != canNum_[3]) {
            return false;
        }
    }

    for (size_t i = 0; i < msg1.size(); i++) {
        txbuf_[board][3 + i * 10] = msg1[i].id;
        txbuf_[board][3 + i * 10 + 1] = msg1[i].id >> 8;
        for (int j = 0; j < 8; j++) {
            txbuf_[board][3 + i * 10 + 2 + j] = msg1[i].data[j];
        }
    }

    for (size_t i = 0; i < msg2.size(); i++) {
        txbuf_[board][msg1.size() * 10 + 3 + i * 10] = msg2[i].id;
        txbuf_[board][msg1.size() * 10 + 3 + i * 10 + 1] = msg2[i].id >> 8;
        for (int j = 0; j < 8; j++) {
            txbuf_[board][msg1.size() * 10 + 3 + i * 10 + 2 + j] = msg2[i].data[j];
        }
    }

    uint32_t spiChecksumTemp;
    spiChecksumTemp = XorChecksum(txbuf_[board], txbuf_[board].size() - 5);
    txbuf_[board][txbuf_[board].size() - 5] = (spiChecksumTemp >> 0) & 0xFF;  // LSB
    txbuf_[board][txbuf_[board].size() - 4] = (spiChecksumTemp >> 8) & 0xFF;
    txbuf_[board][txbuf_[board].size() - 3] = (spiChecksumTemp >> 16) & 0xFF;
    txbuf_[board][txbuf_[board].size() - 2] = (spiChecksumTemp >> 24) & 0xFF;  // MSB

    return true;
}

std::vector<CanData> SpiToCanV3::GetMessage(int board, int canIdx) const
{
    return rxCan_[board * 2 + canIdx];  //
}

void SpiToCanV3::Transfer()
{
    if (isOk_ == false) return;
    ShowFrame(txbuf_, true);

    // 帧头校验
    int looptime = 3;
    while(looptime--)
    {
        for (int i = 0; i < 2; i++) {
            spi_[i].WriteAndRead(txbuf_[i].data(), rxbuf_[i].data(), txbuf_[i].size());
        }
        // TODO:暂时进行简单校验
        if (rxbuf_[0][0] == 0xAA && rxbuf_[1][0] == 0xAA){
            break;
        }
        LOG_WARN("Check SPI Frame Head Error. looptime: {}", looptime);
        ShowFrame(rxbuf_);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    if(looptime < 0)
    {
        LOG_ERROR("Get SPI Data Error.");
        return;
    } 
    ShowFrame(rxbuf_);
    ErrorCheck();
}

uint32_t SpiToCanV3::XorChecksum(std::vector<u8> &data, uint32_t len)
{
    uint32_t *doubleWordData = (uint32_t *)data.data();
    uint32_t doubleWordlen = len / 4;

    uint32_t ret = 0;
    for (size_t i = 0; i < doubleWordlen; i++) {
        ret = ret ^ doubleWordData[i];
    }

    ret += 0xAA;
    return ret;
}

void SpiToCanV3::InitSpi(const std::array<std::string, 2> &spiDev, u32 spiSpd)
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

void SpiToCanV3::ErrorCheck()
{
    bool chkErr = false;

    for (int i = 0; i < 2; i++) {
        uint32_t expectChecksum = XorChecksum(rxbuf_[i], rxbuf_[i].size() - 5);
        uint32_t checkSum = 0;
        checkSum = rxbuf_[i][rxbuf_[i].size() - 5];
        checkSum |= rxbuf_[i][rxbuf_[i].size() - 4] << 8;
        checkSum |= rxbuf_[i][rxbuf_[i].size() - 3] << 16;
        checkSum |= rxbuf_[i][rxbuf_[i].size() - 2] << 24;
        if (expectChecksum != checkSum) {
            LOG_WARN("board {} spi checksum error, actul:{:X}, except:{:X}", i, checkSum, expectChecksum);

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

    if (chkErr == true || isOk_ == false) {
        // 清空rxCan_，避免返回旧数据导致误判电机在线
        rxCan_ = {};
        return;
    }

    if (rxbuf_[0][0] != 0xAA || rxbuf_[0][rxbuf_[0].size() - 1] != 0xBB) {
        return;
    }
    if (rxbuf_[1][0] != 0xAA || rxbuf_[1][rxbuf_[1].size() - 1] != 0xBB) {
        return;
    }

    rxCan_ = {};

    for (std::size_t i = 0; i < canNum_[0]; i++) {
        CanData oneCan;
        oneCan.id = rxbuf_[0][i * 10 + 3];
        oneCan.id |= rxbuf_[0][i * 10 + 4] << 8;
        for (int j = 0; j < 8; j++) {
            oneCan.data[j] = rxbuf_[0][i * 10 + 5 + j];
        }
        rxCan_[0].push_back(oneCan);
    }

    int offset = canNum_[0] * 10;
    for (std::size_t i = 0; i < canNum_[0]; i++) {
        CanData oneCan;
        oneCan.id = rxbuf_[0][offset + i * 10 + 3];
        oneCan.id |= rxbuf_[0][offset + i * 10 + 4] << 8;
        for (int j = 0; j < 8; j++) {
            oneCan.data[j] = rxbuf_[0][offset + i * 10 + 5 + j];
        }
        rxCan_[1].push_back(oneCan);
    }

    for (std::size_t i = 0; i < canNum_[0]; i++) {
        CanData oneCan;
        oneCan.id = rxbuf_[1][i * 10 + 3];
        oneCan.id |= rxbuf_[1][i * 10 + 4] << 8;
        for (int j = 0; j < 8; j++) {
            oneCan.data[j] = rxbuf_[1][i * 10 + 5 + j];
        }
        rxCan_[2].push_back(oneCan);
    }

    offset = canNum_[0] * 10;
    for (std::size_t i = 0; i < canNum_[0]; i++) {
        CanData oneCan;
        oneCan.id = rxbuf_[1][offset + i * 10 + 3];
        oneCan.id |= rxbuf_[1][offset + i * 10 + 4] << 8;
        for (int j = 0; j < 8; j++) {
            oneCan.data[j] = rxbuf_[1][offset + i * 10 + 5 + j];
        }
        rxCan_[3].push_back(oneCan);
    }
}
void SpiToCanV3::ClearBuf() {}

void SpiToCanV3::ShowFrame(std::array<std::vector<u8>, 2>& buf_, bool ShowSendFrame){
    if (access("/tmp/qr_show_frame.enable", F_OK) != 0) {
        return;
    }
    auto PrintSingleFrame = [](const std::vector<u8> &buffer, const std::string &name) {
        std::stringstream ss;
        ss << name << " [" << buffer.size() << "]: ";
        for (size_t i = 0; i < buffer.size(); ++i) {
            ss << std::hex << std::uppercase << std::setw(2) << std::setfill('0')
               << static_cast<int>(buffer[i]) << " ";
            if ((i + 1) % 16 == 0) ss << "\n";
        }
        LOG_INFO(ss.str());
    };

    std::string frameName = ShowSendFrame ? "txbuf_" : "rxbuf_";
    for(auto &v:buf_){
        PrintSingleFrame(v, frameName + "[" + std::to_string(&v - buf_.data()) + "]");
    }
}
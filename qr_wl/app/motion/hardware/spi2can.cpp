#include "spi2can.hpp"

#include <fcntl.h>  //Needed for SPI port
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>  //Needed for SPI port

#include <chrono>

#include "baseline.hpp"

using namespace ::std;
static constexpr int SPI2CAN_SIZE = sizeof(SPI_TO_CAN_S);
static constexpr int CHECKSUM_SIZE = SPI2CAN_SIZE - 4;
static unsigned char spi_mode = SPI_MODE_0;
static unsigned char spi_bits_per_word = 8;
static unsigned int spi_speed = 3000000;
static uint8_t lsb = 0;

SpiToCan::SpiToCan(const std::array<std::string, 2> &spiDev, u32 spiSpd)
{
    InitSpi(spiDev, spiSpd);
    motorSta_.fill(false);
    legNameMap_[0] = "front right";
    legNameMap_[1] = "front  left";
    legNameMap_[2] = "rear  right";
    legNameMap_[3] = "rear   left";
}

void SpiToCan::AddMessage(int board, const SPI_TO_CAN_S &msg) { sendBuf[board] = msg; }

SPI_TO_CAN_S SpiToCan::GetMessage(int board) const { return recvBuf[board]; }

void SpiToCan::Transfer()
{
    if (initOk_ == false) return;
    if (isCheckErr_ == true) return;

    sendBuf[0].checksum = XorChecksum((uint8_t *)&sendBuf[0], CHECKSUM_SIZE);
    sendBuf[1].checksum = XorChecksum((uint8_t *)&sendBuf[1], CHECKSUM_SIZE);

    for (int i = 0; i < 2; i++) {
        spi_[i].WriteAndRead((u8 *)&sendBuf[i], (u8 *)&recvBuf[i], SPI2CAN_SIZE);
    }
    TimeoutCheck();
}

uint32_t SpiToCan::XorChecksum(uint8_t *data, uint32_t len)
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

void SpiToCan::InitSpi(const std::array<std::string, 2> &spiDev, u32 spiSpd)
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

    initOk_ = true;
}

void SpiToCan::TimeoutCheck()
{
    bool chkErr = false;
    for (int i = 0; i < 2; i++) {
        uint32_t expectChecksum = XorChecksum((uint8_t *)&recvBuf[i], CHECKSUM_SIZE);
        if (expectChecksum != recvBuf[i].checksum) {
            LOG_WARN("board {} spi checksum error, actul:{:X}, except:{:X}", i, recvBuf[i].checksum, expectChecksum);
            chkErr = true;
        }
    }

    // 10个里面5个错误，表示错误超出允许值
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
        isCheckErr_ = true;
    }

    if (chkErr == true) {
        return;
    }

    auto nowTime = TimerTools::GetNowTickMs();
    // leg12
    if (recvBuf[0].payload.data[0][0] != 0xFF) {
        leg[0].motor[0].lastRecv = nowTime;
        leg[0].motor[0].timeout = false;
        leg[0].motor[0].recvCnt++;
    }
    if (recvBuf[0].payload.data[1][0] != 0xFF) {
        leg[0].motor[1].lastRecv = nowTime;
        leg[0].motor[1].timeout = false;
        leg[0].motor[1].recvCnt++;
    }

    if (recvBuf[0].payload.data[2][0] != 0xFF) {
        leg[0].motor[2].lastRecv = nowTime;
        leg[0].motor[2].timeout = false;
        leg[0].motor[2].recvCnt++;
    }
    if (recvBuf[0].payload.data[3][0] != 0xFF) {
        leg[1].motor[0].lastRecv = nowTime;
        leg[1].motor[0].timeout = false;
        leg[1].motor[0].recvCnt++;
    }

    if (recvBuf[0].payload.data[4][0] != 0xFF) {
        leg[1].motor[1].lastRecv = nowTime;
        leg[1].motor[1].timeout = false;
        leg[1].motor[1].recvCnt++;
    }

    if (recvBuf[0].payload.data[5][0] != 0xFF) {
        leg[1].motor[2].lastRecv = nowTime;
        leg[1].motor[2].timeout = false;
        leg[1].motor[2].recvCnt++;
    }
    // leg34
    if (recvBuf[1].payload.data[0][0] != 0xFF) {
        leg[2].motor[0].lastRecv = nowTime;
        leg[2].motor[0].timeout = false;
        leg[2].motor[0].recvCnt++;
    }

    if (recvBuf[1].payload.data[1][0] != 0xFF) {
        leg[2].motor[1].lastRecv = nowTime;
        leg[2].motor[1].timeout = false;
        leg[2].motor[1].recvCnt++;
    }

    if (recvBuf[1].payload.data[2][0] != 0xFF) {
        leg[2].motor[2].lastRecv = nowTime;
        leg[2].motor[2].timeout = false;
        leg[2].motor[2].recvCnt++;
    }

    if (recvBuf[1].payload.data[3][0] != 0xFF) {
        leg[3].motor[0].lastRecv = nowTime;
        leg[3].motor[0].timeout = false;
        leg[3].motor[0].recvCnt++;
    }

    if (recvBuf[1].payload.data[4][0] != 0xFF) {
        leg[3].motor[1].lastRecv = nowTime;
        leg[3].motor[1].timeout = false;
        leg[3].motor[1].recvCnt++;
    }

    if (recvBuf[1].payload.data[5][0] != 0xFF) {
        leg[3].motor[2].lastRecv = nowTime;
        leg[3].motor[2].timeout = false;
        leg[3].motor[2].recvCnt++;
    }

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 3; j++) {
            if (nowTime - leg[i].motor[j].lastRecv > 300) {
                if (leg[i].motor[j].timeout == false) {
                    leg[i].motor[j].timeout = true;
                    motorSta_(i, j) = false;
                    legNameMap_[0] = "front right";
                    LOG_ERROR("leg:{}, motor:{}, comm lose, {}", legNameMap_[i], j, nowTime - leg[i].motor[j].lastRecv);
                }
            } else {
                motorSta_(i, j) = true;
            }

            leg[i].motor[j].sendCnt++;
            if ((leg[i].motor[j].sendCnt % 10000) == 0) {
                if ((motorSta_(i, j) == true) && (leg[i].motor[j].sendCnt - leg[i].motor[j].recvCnt > 100)) {
                    LOG_WARN("leg:{}, motor:{}, comm bad quality, diff:{}", i + 1, j + 1, leg[i].motor[j].sendCnt - leg[i].motor[j].recvCnt);
                }

                leg[i].motor[j].sendCnt = 0;
                leg[i].motor[j].recvCnt = 0;
            }
        }
    }
}

void SpiToCan::ResetTimer()
{
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 3; j++) {
            leg[i].motor[j].timeout = false;
            leg[i].motor[j].lastRecv = TimerTools::GetNowTickMs();
        }
    }
}

void SpiToCan::ClearBuf()
{
    for (int i = 0; i < 2; i++) {
        recvBuf[i].Clear();
        sendBuf[i].Clear();
    }
}

Mat43<bool> SpiToCan::GetMotorState() const { return motorSta_; }

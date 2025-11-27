
#include "spi.hpp"

#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>

#include "baseline.hpp"

SpiMaster::SpiMaster(SpiMaster &&other) noexcept
{
    swap(other);
    other.fd_ = -1;
}
SpiMaster &SpiMaster::operator=(SpiMaster &&rhs) noexcept
{
    if (this != &rhs) {
        swap(rhs);
        rhs.fd_ = -1;
    }
    return *this;
}
void SpiMaster::swap(const SpiMaster &other)
{
    fd_ = other.fd_;
    devName_ = other.devName_;
    mode_ = other.mode_;
    speed_ = other.speed_;
    bitsPerWord_ = other.bitsPerWord_;
    lsbFirst_ = other.lsbFirst_;
}

SpiMaster::~SpiMaster() { Close(); };

bool SpiMaster::Open(const std::string &pathname)
{
    devName_ = pathname;
    fd_ = ::open(pathname.c_str(), O_RDWR);
    if (fd_ < 0) {
        LOG_ERROR("SPI Couldn't open {}", pathname);
        return false;
    }
    return SetParam(SPI_MODE_0, 3000000, 8, 0);
}

void SpiMaster::Close()
{
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

bool SpiMaster::SetSpeed(u32 spd)
{
    if ((fd_ >= 0) && (::ioctl(fd_, SPI_IOC_WR_MAX_SPEED_HZ, &spd) < 0)) {
        LOG_ERROR("SPI_IOC_WR_MAX_SPEED_HZ:{}", GetSysError(errno));
        return false;
    }
    speed_ = spd;
    return true;
}
bool SpiMaster::SetMode(u32 mode)
{
    if ((fd_ >= 0) && (::ioctl(fd_, SPI_IOC_WR_MODE, &mode) < 0)) {
        LOG_ERROR("SPI_IOC_WR_MODE:{}", GetSysError(errno));
        return false;
    }
    mode_ = mode;
    return true;
}
bool SpiMaster::SetBitsPerWord(u32 num)
{
    if ((fd_ >= 0) && (::ioctl(fd_, SPI_IOC_WR_BITS_PER_WORD, &num) < 0)) {
        LOG_ERROR("SPI_IOC_WR_BITS_PER_WORD:{}", GetSysError(errno));
        return false;
    }
    bitsPerWord_ = num;
    return true;
}
bool SpiMaster::SetLsb(u32 lsbFirst)
{
    if ((fd_ >= 0) && (::ioctl(fd_, SPI_IOC_WR_LSB_FIRST, &lsbFirst) < 0)) {
        LOG_ERROR("SPI_IOC_WR_LSB_FIRST:{}", GetSysError(errno));
        return false;
    }
    lsbFirst_ = lsbFirst;
    return true;
}
bool SpiMaster::SetParam(u32 mode, u32 speed, u32 bitsPerWord, u32 lsb)
{
    RETURN_IF(SetMode(mode));
    RETURN_IF(SetSpeed(speed));
    RETURN_IF(SetBitsPerWord(bitsPerWord));
    RETURN_IF(SetLsb(lsb));
    return true;
}
void SpiMaster::WriteAndRead(const std::vector<u8> &tx, std::vector<u8> *rx) const { WriteAndRead(tx.data(), rx->data(), tx.size()); }

void SpiMaster::WriteAndRead(const u8 *tx, u8 *rx, u16 len) const
{
    struct spi_ioc_transfer spi_message = {};

    spi_message.bits_per_word = bitsPerWord_;
    spi_message.cs_change = 0;
    spi_message.delay_usecs = 10;
    spi_message.len = len;
    spi_message.rx_buf = (uint64_t)rx;
    spi_message.tx_buf = (uint64_t)tx;

    ::ioctl(fd_, SPI_IOC_MESSAGE(1), &spi_message);
}
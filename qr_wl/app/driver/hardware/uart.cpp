
#include "uart.hpp"

#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <sys/types.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#include "baseline.hpp"
using namespace std;

/**
 * @description: 构造函数，构造时必须指定串口设备
 * @param &pathname
 * @return {}
 */
Uart::Uart(const std::string &devName) : uartName_(devName) { isOpen_ = Open(); }
Uart::Uart(Uart &&other) noexcept
{
    swap(other);
    other.uartFd = -1;
}
Uart &Uart::operator=(Uart &&rhs) noexcept
{
    if (this != &rhs) {
        swap(rhs);
        rhs.uartFd = -1;
    }
    return *this;
}
void Uart::swap(const Uart &other)
{
    uartFd = other.uartFd;
    blockCnt_ = other.blockCnt_;
}

bool Uart::Open()
{
    uartFd = ::open(uartName_.c_str(), O_RDWR | O_NOCTTY);
    if (uartFd < 0) {
        // LOG_ERROR("Uart open error: {}, {}", uartName_, strerror(errno));
        return false;
    }
    LOG_INFO("Uart open success:{}", uartName_);
    return true;
}

void Uart::Close()
{
    ::close(uartFd);
    uartFd = 0;
    isOpen_ = false;
}

bool Uart::IsOpen() const { return isOpen_; }

/**
 * @description: 设备是否存在可用/有效
 * @return {}
 */
bool Uart::IsValid() const
{
    if (::access(uartName_.c_str(), F_OK | W_OK | R_OK) < 0) {
        return false;
    }
    return true;
}

bool Uart::SetParam(int nSpeed, int nBits, char nEvent, int nStop)
{
    if (IsOpen() == false) {
        return false;
    }

    struct termios newtio;
    if (tcgetattr(uartFd, &newtio) != 0) {
        LOG_ERROR("Uart::SetParam get error: {}", strerror(errno));
        return false;
    }

    bzero(&newtio, sizeof(newtio));
    newtio.c_cflag |= CLOCAL | CREAD;
    newtio.c_cflag &= ~CSIZE;
    switch (nBits) {
        case 7:
            newtio.c_cflag |= CS7;
            break;
        case 8:
            newtio.c_cflag |= CS8;
            break;
    }
    switch (nEvent) {
        case 'o':
        case 'O':
            newtio.c_cflag |= PARENB;
            newtio.c_cflag |= PARODD;
            newtio.c_iflag |= (INPCK | ISTRIP);
            break;
        case 'e':
        case 'E':
            newtio.c_iflag |= (INPCK | ISTRIP);
            newtio.c_cflag |= PARENB;
            newtio.c_cflag &= ~PARODD;
            break;
        case 'n':
        case 'N':
            newtio.c_cflag &= ~PARENB;
            break;
        default:
            break;
    }

    /*设置波特率*/
    switch (nSpeed) {
        case 2400:
            cfsetispeed(&newtio, B2400);
            cfsetospeed(&newtio, B2400);
            break;
        case 4800:
            cfsetispeed(&newtio, B4800);
            cfsetospeed(&newtio, B4800);
            break;
        case 9600:
            cfsetispeed(&newtio, B9600);
            cfsetospeed(&newtio, B9600);
            break;
        case 115200:
            cfsetispeed(&newtio, B115200);
            cfsetospeed(&newtio, B115200);
            break;
        case 460800:
            cfsetispeed(&newtio, B460800);
            cfsetospeed(&newtio, B460800);
            break;
        default:
            cfsetispeed(&newtio, B9600);
            cfsetospeed(&newtio, B9600);
            break;
    }

    if (nStop == 1) {
        newtio.c_cflag &= ~CSTOPB;
    } else if (nStop == 2) {
        newtio.c_cflag |= CSTOPB;
    }

    newtio.c_cc[VTIME] = 0;
    newtio.c_cc[VMIN] = blockCnt_;

    newtio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    if ((tcsetattr(uartFd, TCSANOW, &newtio)) != 0) {
        LOG_ERROR("Uart::SetParam set error: {}", strerror(errno));
        return false;
    }

    return true;
}

/**
 * @description: 设置阻塞个数，阻塞到该数量数据后，才返回。
 * @param cnt
 * @return {}
 */
bool Uart::SetBlockCnt(int cnt)
{
    if (IsOpen() == false) {
        return false;
    }
    blockCnt_ = cnt;
    struct termios oldtio;
    if (tcgetattr(uartFd, &oldtio) != 0) {
        LOG_ERROR("Uart::SetBlockCnt get error: {}", strerror(errno));
        return false;
    }

    oldtio.c_cc[VMIN] = blockCnt_;

    if ((tcsetattr(uartFd, TCSANOW, &oldtio)) != 0) {
        LOG_ERROR("Uart::SetBlockCnt set error: {}", strerror(errno));
        return false;
    }
    return true;
}

/**
 * @description: 串口读
 * @param ms 超时时间，默认2ms
 * @return {}
 */
std::vector<u8> Uart::Read(int ms)
{
    std::vector<u8> retData;
    if (IsOpen() == false) {
        return retData;
    }

    // 先设置为非阻塞模式
    int flags = ::fcntl(uartFd, F_GETFL, 0);
    flags |= O_NONBLOCK;
    ::fcntl(uartFd, F_SETFL, flags);

    //  select读取
    fd_set rset;
    FD_ZERO(&rset);
    FD_SET(uartFd, &rset);
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = ms * 1000;  // 2ms

    int ret = select(uartFd + 1, &rset, NULL, NULL, &tv);
    if (ret == 0) {
        return retData;
    } else if (ret > 0) {
        u8 oneData;
        while (::read(uartFd, &oneData, 1) > 0) {
            retData.push_back(oneData);
        }
    } else {
        LOG_INFO("uart select error");
    }

    return retData;
}

/**
 * @description: 串口写，阻塞
 * @return {}
 */
bool Uart::Write(const std::vector<u8> &data)
{
    if (IsOpen() == false) {
        return false;
    }
    // 先设置为阻塞模式
    int flags = ::fcntl(uartFd, F_GETFL, 0);
    flags &= ~O_NONBLOCK;
    ::fcntl(uartFd, F_SETFL, flags);

    auto ret = ::write(uartFd, data.data(), data.size());
    if (ret < 0) {
        return false;
    }
    return true;
}


#pragma once
#include <string>
#include <vector>

#include "baseline.hpp"

enum class UartOpt
{
    block,
    noBlock,
};

class Uart
{
public:
    Uart(const std::string &devName);
    virtual ~Uart() { Close(); }
    Uart(const Uart &) = delete;
    Uart &operator=(const Uart &) = delete;
    Uart(Uart &&other) noexcept;  // 但可以移动
    Uart &operator=(Uart &&rhs) noexcept;
    void swap(const Uart &other);

public:
    bool Open();
    bool SetParam(int nSpeed, int nBits, char nEvent, int nStop);
    bool SetBlockCnt(int cnt);
    void Close();
    std::vector<u8> Read(int ms = 2);
    bool Write(const std::vector<u8> &data);
    bool IsOpen() const;
    bool IsValid() const;

private:
    int uartFd{0};
    bool isOpen_{false};
    std::string uartName_;
    u32 blockCnt_ = 1;  // 接收到多少个数据才触发
};
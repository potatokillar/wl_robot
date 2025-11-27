
#pragma once
#include <string>
#include <vector>

#include "baseline.hpp"

class SpiMaster
{
public:
    SpiMaster() = default;
    virtual ~SpiMaster();
    SpiMaster(const SpiMaster& other) = delete;
    SpiMaster& operator=(const SpiMaster& rhs) = delete;
    SpiMaster(SpiMaster&& other) noexcept;
    SpiMaster& operator=(SpiMaster&& rhs) noexcept;
    void swap(const SpiMaster& other);

public:
    bool Open(const std::string& pathname);
    void Close();
    bool SetSpeed(u32 spd);
    bool SetMode(u32 mode);
    bool SetBitsPerWord(u32 num);
    bool SetLsb(u32 lsbFirst);
    bool SetParam(u32 mode, u32 speed, u32 bitsPerWord, u32 lsb);
    void WriteAndRead(const std::vector<u8>& tx, std::vector<u8>* rx) const;
    void WriteAndRead(const u8* tx, u8* rx, u16 len) const;

private:
    int fd_{-1};
    std::string devName_;
    u32 mode_{0};
    u32 speed_{0};
    u32 bitsPerWord_{0};
    u32 lsbFirst_{0};
};

class SpiSlave
{
};
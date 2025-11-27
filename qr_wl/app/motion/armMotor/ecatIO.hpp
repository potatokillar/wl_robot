
#pragma once
#include "baseline.hpp"

class EcatIO
{
public:
    EcatIO(int slc);
    void PreOpInit();
    void OpInit() {};  // 暂没有op下的初始化
    void SetIoOut(u16 set);
    void SetIoOut(int idx, bool set);

    bool GetIoIn(int idx);
    u16 GetIoIn();

private:
    u32 GetPdoMapData(u16 index, u8 subIndex, u8 len);
    int slc_;
};
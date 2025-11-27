
#include "ecatMotor.hpp"

#include <soem/ethercat.h>

#include "baseline.hpp"
#include "ecatBase.hpp"

using namespace ethercat;

#pragma pack(1)
// 发送给从机的默认PDO
struct RPDO1_PV
{
    s32 position;
    u32 ioOut;
    u16 ctrlword;
};

// 从机发出的默认PDO
struct TPDO1_PV
{
    u16 statusWord;
    u16 errorCode;
    s32 position;
    s32 velocity;
    u32 ioIn;
    u32 ioOut;
    s32 torque;
};
#pragma pack()

EcatMotor::EcatMotor(int slc) : slc_(slc)
{
    circleCnt_ = std::pow(2, 19);
    biasCnt_ = 262144;  // 绝对值编码器是固定值
}

void EcatMotor::PreOpInit()
{
    u32 mapData;
    int len = sizeof(mapData);
    // int wkc = 0;
    // 设置为freeRun模式
    u16 data16 = 0;
    int len16 = 2;
    ec_slave[slc_].blockLRW = 1;
    mapData = 0;
    ec_SDOwrite(slc_, 0x1C12, 0x00, FALSE, len, &mapData, EC_TIMEOUTRXM);
    ec_SDOwrite(slc_, 0x1C13, 0x00, FALSE, len, &mapData, EC_TIMEOUTRXM);
    // 设置RPDO1
    mapData = 0;
    ec_SDOwrite(slc_, RPDO1_map, 0x00, FALSE, len, &mapData, EC_TIMEOUTRXM);

    mapData = GetPdoMapData(0x607A, 0x0, 4);
    ec_SDOwrite(slc_, RPDO1_map, 0x01, FALSE, len, &mapData, EC_TIMEOUTRXM);
    mapData = GetPdoMapData(0x60FE, 0x0, 4);
    ec_SDOwrite(slc_, RPDO1_map, 0x02, FALSE, len, &mapData, EC_TIMEOUTRXM);
    mapData = GetPdoMapData(0x6040, 0x0, 2);
    ec_SDOwrite(slc_, RPDO1_map, 0x04, FALSE, len, &mapData, EC_TIMEOUTRXM);

    mapData = 3;
    ec_SDOwrite(slc_, RPDO1_map, 0x00, FALSE, len, &mapData, EC_TIMEOUTRXM);

    // 设置TPDO1
    mapData = 0;
    ec_SDOwrite(slc_, TPDO1_map, 0x00, FALSE, len, &mapData, EC_TIMEOUTRXM);
    mapData = GetPdoMapData(0x6041, 0x0, 2);
    ec_SDOwrite(slc_, TPDO1_map, 0x01, FALSE, len, &mapData, EC_TIMEOUTRXM);
    mapData = GetPdoMapData(0x603F, 0x0, 2);
    ec_SDOwrite(slc_, TPDO1_map, 0x02, FALSE, len, &mapData, EC_TIMEOUTRXM);
    mapData = GetPdoMapData(0x6064, 0x0, 4);
    ec_SDOwrite(slc_, TPDO1_map, 0x03, FALSE, len, &mapData, EC_TIMEOUTRXM);
    mapData = GetPdoMapData(0x606C, 0x0, 4);
    ec_SDOwrite(slc_, TPDO1_map, 0x04, FALSE, len, &mapData, EC_TIMEOUTRXM);
    mapData = GetPdoMapData(0x60FD, 0x0, 4);
    ec_SDOwrite(slc_, TPDO1_map, 0x05, FALSE, len, &mapData, EC_TIMEOUTRXM);
    mapData = GetPdoMapData(0x60FE, 0x0, 4);
    ec_SDOwrite(slc_, RPDO1_map, 0x06, FALSE, len, &mapData, EC_TIMEOUTRXM);
    mapData = GetPdoMapData(0x6077, 0x0, 4);
    ec_SDOwrite(slc_, RPDO1_map, 0x07, FALSE, len, &mapData, EC_TIMEOUTRXM);

    mapData = 7;
    ec_SDOwrite(slc_, TPDO1_map, 0x00, FALSE, len, &mapData, EC_TIMEOUTRXM);

    mapData = 0x1600;
    ec_SDOwrite(slc_, 0x1C12, 0x01, FALSE, len, &mapData, EC_TIMEOUTRXM);
    mapData = 1;
    ec_SDOwrite(slc_, 0x1C12, 0x00, FALSE, len, &mapData, EC_TIMEOUTRXM);

    mapData = TPDO1_map;
    ec_SDOwrite(slc_, 0x1C13, 0x01, FALSE, len, &mapData, EC_TIMEOUTRXM);
    mapData = 1;
    ec_SDOwrite(slc_, 0x1C13, 0x00, FALSE, len, &mapData, EC_TIMEOUTRXM);

    ec_SDOwrite(slc_, 0x1C32, 0x01, FALSE, len16, &data16, EC_TIMEOUTRXM);
}

void EcatMotor::OpInit()
{
    s32 mapData;
    int len = sizeof(mapData);
    // 将当前位置写入位置命令对象字典
    ec_SDOread(slc_, 0x6064, 0x00, FALSE, &len, &mapData, EC_TIMEOUTRXM);
    ec_SDOwrite(slc_, 0x607A, 0x00, FALSE, len, &mapData, EC_TIMEOUTRXM);
    RPDO1_PV* output = (RPDO1_PV*)ec_slave[slc_].outputs;
    output->position = mapData;
    output->ctrlword = 0x80;
}

/**
 * @description: 设置位置
 * @param q 弧度
 * @return {}
 */
void EcatMotor::SetPosition(double q)
{
    s32 pulse = q * circleCnt_ / (2 * M_PI) + biasCnt_;  // 转成驱动器的cnt
    RPDO1_PV* output = (RPDO1_PV*)ec_slave[slc_].outputs;
    output->position = pulse;
}

/**
 * @description: 返回关节位置
 * @return {} 弧度
 */
double EcatMotor::GetPosition()
{
    TPDO1_PV* input = (TPDO1_PV*)ec_slave[slc_].inputs;
    return (2 * M_PI) * (input->position - biasCnt_) / circleCnt_;
    ;
}

s32 EcatMotor::GetTorque()
{
    TPDO1_PV* input = (TPDO1_PV*)ec_slave[slc_].inputs;
    return input->torque;
}

/**
 * @description:
 * @return {}
 */
s32 EcatMotor::GetVelocity()
{
    TPDO1_PV* input = (TPDO1_PV*)ec_slave[slc_].inputs;
    // return input->velocity;
    return (2 * M_PI) * input->velocity / circleCnt_;
}

void EcatMotor::SetCtrlWord(u32 set)
{
    RPDO1_PV* output = (RPDO1_PV*)ec_slave[slc_].outputs;
    output->ctrlword = set;
}
u32 EcatMotor::GetStatusWord()
{
    TPDO1_PV* input = (TPDO1_PV*)ec_slave[slc_].inputs;
    return input->statusWord;
}
void EcatMotor::SetAccVelocity(s32 pulse) { ec_SDOwrite(slc_, 0x6083, 0x00, FALSE, 4, &pulse, EC_TIMEOUTRXM); }
void EcatMotor::SetDecVelocity(s32 pulse) { ec_SDOwrite(slc_, 0x6084, 0x00, FALSE, 4, &pulse, EC_TIMEOUTRXM); }

u32 EcatMotor::GetErrorCode()
{
    TPDO1_PV* input = (TPDO1_PV*)ec_slave[slc_].inputs;
    if (input->errorCode != errCode_) {
        LOG_ERROR("slave {}, error: 0x{:02X}", slc_, input->errorCode);
        errCode_ = input->errorCode;
    }
    return input->errorCode;
}

void EcatMotor::ClearErrorCode() { SetCtrlWord(0x80); }

/**
 * @description: 零差运控的电机，最多两个IO设置
 * @param io
 * @return {}
 */
void EcatMotor::SetIoOut(u32 io)
{
    io = (io & 0x03) << 16;  // bit16:17是对应的两个io实际值
    RPDO1_PV* output = (RPDO1_PV*)ec_slave[slc_].outputs;
    output->ioOut = io;
}
u32 EcatMotor::GetIoIn()
{
    TPDO1_PV* input = (TPDO1_PV*)ec_slave[slc_].inputs;
    // STO占用了一路，目前还不能确定是bit16还是bit17
    return (input->ioIn >> 16) & 0x01;
}

void EcatMotor::SetMode(ServoMode mode)
{
    uint8_t modeNum = Enum2Num(mode);
    int len = sizeof(modeNum);
    ec_SDOwrite(slc_, 0x6060, 0x00, FALSE, len, &modeNum, EC_TIMEOUTRXM);
}

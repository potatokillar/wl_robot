#include <gtest/gtest.h>

// 其它类
#include "baseline.hpp"
#include "canBridge.hpp"
#include "canDeal.hpp"
#include "stub.h"
#include "uart.hpp"

using namespace ::std;

#if 1

constexpr int TEST1NUM = 5;
// #define TEST1NUM 5
static vector<u8> readData;
std::mutex rxMutex;
constexpr uint8_t LOCAL_ADDRESS = 1;
// 等于0，没有新的数据（没有新的数据也不清空，只是改为发送一个空的数据包）。
uint8_t readDataFlag = 0;

/**
 * @brief 提供错误的校验和的函数
 *
 * @param TxData
 */
void PutTestFifoErr(uint8_t TxData[8])
{
    uint8_t dataBuff[13] = {0};
    const uint8_t StdId = 1;
    uint8_t add = 0;
    dataBuff[0] = 0xAA;
    dataBuff[1] = 10;
    dataBuff[2] = StdId >> 8;
    dataBuff[3] = StdId;
    dataBuff[4] = TxData[0];
    dataBuff[5] = TxData[1];
    dataBuff[6] = TxData[2];
    dataBuff[7] = TxData[3];
    dataBuff[8] = TxData[4];
    dataBuff[9] = TxData[5];
    dataBuff[10] = TxData[6];
    dataBuff[11] = TxData[7];
    for (size_t i = 0; i < 10; i++) {
        add += dataBuff[2 + i];
    }
    dataBuff[12] = (add + 1);
    readDataFlag = 1;
    readData.insert(readData.end(), std::begin(dataBuff), std::end(dataBuff));
}
/**
 * @brief 提供正确的校验和的函数
 *
 * @param TxData
 */
void PutTestFifo(uint8_t TxData[8])
{
    uint8_t dataBuff[13] = {0};
    const uint8_t StdId = 1;
    uint8_t add = 0;
    dataBuff[0] = 0xAA;
    dataBuff[1] = 10;
    dataBuff[2] = StdId >> 8;
    dataBuff[3] = StdId;
    dataBuff[4] = TxData[0];
    dataBuff[5] = TxData[1];
    dataBuff[6] = TxData[2];
    dataBuff[7] = TxData[3];
    dataBuff[8] = TxData[4];
    dataBuff[9] = TxData[5];
    dataBuff[10] = TxData[6];
    dataBuff[11] = TxData[7];
    for (size_t i = 0; i < 10; i++) {
        add += dataBuff[2 + i];
    }
    dataBuff[12] = add;
    readDataFlag = 1;
    readData.insert(readData.end(), std::begin(dataBuff), std::end(dataBuff));
}

/* 以下为定时上报的协议，定时上报相关处理在实际运行中为数据上报之后数据自动存储，等待被应用层读取，故发数据组包的入栈可以在应用层调用读取AIP之前执行*/
/*电流检测（功能码03H），索引1,2,3,4.*/
void CanTxmotorCurrent(s16 motorCurrent1, s16 motorCurrent2, s16 motorCurrent3, s16 motorCurrent4)
{
    uint8_t TxData[8];
    // 电机电流的第一个报文
    TxData[0] = 0x03;                  // 功能码
    TxData[1] = 0x01;                  // 数据域，索引
    TxData[2] = (motorCurrent1) >> 8;  // 数据域，一个s16的值
    TxData[3] = motorCurrent1;         // 数据域，一个s16的值
    TxData[4] = 0x02;                  // 一个字节的索引
    TxData[5] = (motorCurrent2) >> 8;  // 数据域，一个s16的值
    TxData[6] = motorCurrent2;         // 数据域，一个s16的值
    TxData[7] = 0;                     // 保留
    PutTestFifo(TxData);

    // 电机电流的第二个报文
    TxData[0] = 0x03;                  // 功能码
    TxData[1] = 0x03;                  // 数据域，索引
    TxData[2] = (motorCurrent3) >> 8;  // 数据域，一个s16的值
    TxData[3] = motorCurrent3;         // 数据域，一个s16的值
    TxData[4] = 0x04;                  // 数据域，索引
    TxData[5] = (motorCurrent4) >> 8;  // 数据域，一个s16的值
    TxData[6] = motorCurrent4;         // 数据域，一个s16的值
    TxData[7] = 0;                     // 保留
    PutTestFifo(TxData);
}

/*电流检测（功能码03H），索引5。*/
void CanTxBatteryPackCurrent(s16 bmsCurrent)
{
    uint8_t TxData[8];

    TxData[0] = 0x03;               // 功能码
    TxData[1] = 0x05;               // 数据域，索引
    TxData[2] = (bmsCurrent) >> 8;  // 数据域，一个s16的值
    TxData[3] = bmsCurrent;         // 数据域，一个s16的值
    TxData[4] = 0x00;               // 保留
    TxData[5] = 0x00;               // 保留
    TxData[6] = 0x00;               // 保留
    TxData[7] = 0x00;               // 保留
    PutTestFifo(TxData);
}

/*5.5.急停检测（功能码04H）*/
void CanTxCheckStopMotor(void)
{
    uint8_t TxData[8];
    TxData[0] = 0x04;  // 功能码
    TxData[1] = 0x01;  // 数据域，1表示按下
    TxData[2] = 0;     // 预留
    TxData[3] = 0;     // 预留
    TxData[4] = 0;     // 预留
    TxData[5] = 0;     // 预留
    TxData[6] = 0;     // 预留
    TxData[7] = 0;     // 预留
    PutTestFifo(TxData);
}

// 电池状态上报（功能码05H）。获取电池包的电量，电流，电压
/**
 * @description: 传输电池包的电流、电压、温度等数据，如果达到阈值则在电池状态位报警
 * @param batteryLevel：使用百分比来表示。传输值为实际值的百分比，用u8来表示0-100。实际上现在的算法不能得到精准的电量
 * @param batteryVoltage：电流，单位10mA
 * @param batteryCurrent：电压，单位10mV
 * @param batteryStatus：01H充电，02H放电，03H温度报警，04H电流报警，05H电压报警。
 * @return {}
 */
void CanTxBatteryPower(u8 batteryLevel, s16 batteryCurrent, s16 batteryVoltage, u8 batteryStatus)
{
    uint8_t TxData[8];
    // uint32_t StdId;
    // // 电机电流的第一个报文
    // StdId = LOCAL_ADDRESS;            // 标识符
    TxData[0] = 0x05;                 // 功能码
    TxData[1] = batteryLevel;         // 数据域，电池电量
    TxData[2] = batteryCurrent >> 8;  // 数据域，电池电压*100后的后8位
    TxData[3] = batteryCurrent;       // 数据域，电池电压*100后的后8位
    TxData[4] = batteryVoltage >> 8;  // 数据域，电池电流*100后的前8位
    TxData[5] = batteryVoltage;       // 数据域，电池电流*100后的后8位
    TxData[6] = batteryStatus;        // 数据域，电池异常状态报警
    TxData[7] = 0;                    // 预留
    PutTestFifo(TxData);
}

/*温度状态上报（功能码06H）。索引3，获取*/
void CanTxPowerPcbTemperature3(u8 bmsTempT1, u8 bmsTempT2, u8 bmsTempT3, u8 errBit)
{
    uint8_t TxData[8];
    // uint32_t StdId;

    // // 电池温度的第一个报文
    // StdId = LOCAL_ADDRESS;  // 标识符
    TxData[0] = 0x06;       // 功能码
    TxData[1] = 0x03;       // 数据域，索引
    TxData[2] = bmsTempT1;  // 数据域，一个u8的值
    TxData[3] = bmsTempT2;  // 数据域，一个u8的值
    TxData[4] = bmsTempT3;  // 数据域，一个u8的值
    TxData[5] = errBit;     // 错误报警位
    TxData[6] = 0;          // 保留
    TxData[7] = 0;          // 保留
    PutTestFifo(TxData);
}
/*温度状态上报（功能码06H）10s一次。索引2*/
void CanTxPowerPcbTemperature2(u8 motorTempT3)
{
    uint8_t TxData[8];
    // uint32_t StdId;

    // // 电池温度的第一个报文
    // StdId = LOCAL_ADDRESS;  // 标识符
    TxData[0] = 0x06;         // 功能码
    TxData[1] = 0x02;         // 数据域，索引
    TxData[2] = motorTempT3;  // 数据域，一个u8的值
    TxData[3] = 0;            // 数据域，一个u8的值
    TxData[4] = 0;            // 数据域，一个u8的值
    TxData[5] = 0;            // 错误报警位
    TxData[6] = 0;            // 保留
    TxData[7] = 0;            // 保留
    PutTestFifo(TxData);
}

/*温度状态上报（功能码06H）10s一次。索引2*/
void CanTxPowerPcbTemperature1(u8 batteryTemperature1, u8 batteryTemperature2, u8 batteryTemperature3, u8 batteryTemperature4)
{
    uint8_t TxData[8];
    // uint32_t StdId;

    // // 电池温度的第一个报文
    // StdId = LOCAL_ADDRESS;  // 标识符
    TxData[0] = 0x06;                 // 功能码
    TxData[1] = 0x01;                 // 数据域，索引
    TxData[2] = batteryTemperature1;  // 数据域，一个u8的值
    TxData[3] = batteryTemperature2;  // 数据域，一个u8的值
    TxData[4] = batteryTemperature3;  // 数据域，一个u8的值
    TxData[5] = batteryTemperature4;  // 错误报警位
    TxData[6] = 0;                    // 保留
    TxData[7] = 0;                    // 保留
    PutTestFifo(TxData);
}

// 查询类协议组包,一下代码需要修改，bool stub_Write函数的测试夹具，在write函数中模拟回复
// 功能码00H,主机查询从机地址的回复
void CanTxReplySlaveAddr(uint8_t slaveAddr)
{
    // 这个功能不用检查数据域
    uint8_t TxData[8];
    TxData[0] = 0x00;       // 功能码
    TxData[1] = slaveAddr;  // 数据域，从机地址
    TxData[2] = 0x00;
    TxData[3] = 0x00;
    TxData[4] = 0;
    TxData[5] = 0;
    TxData[6] = 0;
    TxData[7] = 0;
    PutTestFifo(TxData);
}

/**
 * @description: 电源模块的回复（功能码01H）
 * @param PowerType 电源种类
 * @param Powerindex 电源索引
 * @param PowerStara 电源模块的状态
 * @return {}
 */
void CanTxReplyStartMotorPower(uint8_t PowerType, uint8_t Powerindex, bool PowerStara)
{
    uint8_t TxData[8];

    TxData[0] = 0x01;        // 功能码
    TxData[1] = PowerType;   // 数据域，电源种类
    TxData[2] = Powerindex;  // 数据域，电源索引
    TxData[3] = PowerStara;  // 数据域，现在只有电机模块电源状态,1表示开
    TxData[4] = 0;           // 预留
    TxData[5] = 0;           // 预留
    TxData[6] = 0;           // 预留
    TxData[7] = 0;           // 预留
    PutTestFifo(TxData);
}

/** 电源状态的回复（功能码02H）
 * @description:
 * @param PowerTypeRead:电源种类
 * @param PowerindexRead:电源索引
 * @param PowerStaraRead:电源状态，1开，0关
 * @return 无
 */
void CanTxReplyMotorPowerState(uint8_t PowerTypeRead, uint8_t PowerindexRead, bool PowerStaraRead)
{
    uint8_t TxData[8];
    TxData[0] = 0x02;            // 功能码
    TxData[1] = PowerTypeRead;   // 数据域，电源种类
    TxData[2] = PowerindexRead;  // 数据域，电源索引
    TxData[3] = PowerStaraRead;  // 数据域，现在只有电机模块电源状态,1表示开
    TxData[4] = 0;               // 预留
    TxData[5] = 0;               // 预留
    TxData[6] = 0;               // 预留
    TxData[7] = 0;               // 预留
    PutTestFifo(TxData);
}

/*5.6.运动控制器下发告知关机的回复（功能码07H）,只要收到该回复则完成任务*/
void CanTxReplyOffThePower(void)
{
    uint8_t TxData[8];

    TxData[0] = 0x07;           // 功能码
    TxData[1] = LOCAL_ADDRESS;  // 数据域，索引
    TxData[2] = 0x00;           // 数据域，一个u8的值
    TxData[3] = 0x00;           // 数据域，一个u8的值
    TxData[4] = 0x00;           // 数据域，一个u8的值
    TxData[5] = 0x00;           // 数据域，一个u8的值
    TxData[6] = 0x00;           // 保留
    TxData[7] = 0x00;           // 保留
    PutTestFifo(TxData);
}

// 将readData和相关桩函数写入测试类CanDeal_config_base无效。
// 其原因可能是打桩函数可能会构造一个新类，从而导致readData不是一个内存地址
// 而现在这种全局变量的形式，作用域不合理但内存地址是唯一的

// 串口类接口的桩函数
bool stub_Open(void *pThis, const string &str)
{
    UNUSED(pThis);
    UNUSED(str);
    return true;
}
bool stub_SetParam(void *pThis, int nSpeed, int nBits, char nEvent, int nStop)
{
    UNUSED(pThis);
    UNUSED(nSpeed);
    UNUSED(nBits);
    UNUSED(nEvent);
    UNUSED(nStop);
    return true;
}
bool stub_SetBlockCnt(void *pThis, int cnt)
{
    UNUSED(pThis);
    UNUSED(cnt);
    return true;
}
void stub_Close(void *pThis) { UNUSED(pThis); }

bool stub_Write(void *pThis, const std::vector<u8> &data)
{
    UNUSED(pThis);
    UNUSED(data);

    return 1;
}
std::vector<u8> stub_Read(void *pThis, int ms = 2)
{
    UNUSED(pThis);
    UNUSED(ms);
    vector<u8> a;
    if (readDataFlag == 1) {
        readDataFlag = 0;
        return readData;

    } else {
        return a;
    }
}

/**正确数据的夹具*/
// 构造函数和析构函数，本类构造和析构时执行一次
// SetUp/TearDown函数，每个TEST_F都会执行一次
class CanDeal_config_base : public ::testing::Test
{
protected:
    CanDeal_config_base()
    {
        GetLog().SetLevel(6);
        GetRobotCfgFile().ParseFile("./config/qr-linkV2-3.toml");
        // 对底层函数打桩
        // ，stub.set方法用于替换函数的实现。它接受两个参数：第一个参数是要替换的函数的地址，第二个参数是替换后的函数的地址
        stub.set(ADDR(Uart, Open), stub_Open);
        stub.set(ADDR(Uart, SetParam), stub_SetParam);
        stub.set(ADDR(Uart, SetBlockCnt), stub_SetBlockCnt);
        stub.set(ADDR(Uart, Close), stub_Close);
        stub.set(ADDR(Uart, Write), stub_Write);
        stub.set(ADDR(Uart, Read), stub_Read);
    }
    ~CanDeal_config_base() override {}
    void SetUp() override
    {
        lock_guard<mutex> lock(rxMutex);
        {
            // 正常电池数据
            {
                // 定时上报
            }
        }
        GetCanBridge().Init();
        // can线程10ms运行一次，延时保证它一定执行过，若小于两倍可能导致canBridge未收到canDeal信息
        TimerTools::SleepForMs(30);
    }
    void TearDown() override {}

private:
    Stub stub;
};

// 定时上报，正常数据中的测试
TEST_F(CanDeal_config_base, batteryInfo)
{
    HardwareSensorAPI sensor;
    s16 motorCurrent1, motorCurrent2, motorCurrent3, motorCurrent4;
    s16 bmsCurrent;
    for (size_t i = 0; i < TEST1NUM; i++) {
        if (i == 0) {
            motorCurrent1 = std::numeric_limits<decltype(motorCurrent1)>::min();
            motorCurrent2 = std::numeric_limits<decltype(motorCurrent2)>::min();
            motorCurrent3 = std::numeric_limits<decltype(motorCurrent3)>::min();
            motorCurrent4 = std::numeric_limits<decltype(motorCurrent4)>::min();
            bmsCurrent = std::numeric_limits<decltype(bmsCurrent)>::min();
        } else if (i == 1) {
            motorCurrent1 = std::numeric_limits<decltype(motorCurrent1)>::max();
            motorCurrent2 = std::numeric_limits<decltype(motorCurrent2)>::max();
            motorCurrent3 = std::numeric_limits<decltype(motorCurrent3)>::max();
            motorCurrent4 = std::numeric_limits<decltype(motorCurrent4)>::max();
            bmsCurrent = std::numeric_limits<decltype(bmsCurrent)>::max();
        } else if (i == 2) {
            motorCurrent1 = 0;
            motorCurrent2 = 0;
            motorCurrent3 = 0;
            motorCurrent4 = 0;
            bmsCurrent = 0;
        } else {
            motorCurrent1 = std::rand() % (std::numeric_limits<decltype(motorCurrent4)>::max());
            motorCurrent2 = std::rand() % (std::numeric_limits<decltype(motorCurrent4)>::max());
            motorCurrent3 = std::rand() % (std::numeric_limits<decltype(motorCurrent4)>::max());
            motorCurrent4 = std::rand() % (std::numeric_limits<decltype(motorCurrent4)>::max());
            bmsCurrent = std::rand() % (std::numeric_limits<decltype(motorCurrent4)>::max());
        }
        CanTxmotorCurrent(motorCurrent1, motorCurrent2, motorCurrent3, motorCurrent4);
        CanTxBatteryPackCurrent(bmsCurrent);
        TimerTools::SleepForMs(25);

        // 测试4个电机及电池包电流
        s16 rightFrontLegCurrent = sensor.GetLegCurrent().data;
        s16 leftFrontLegCurrent = sensor.GetLegCurrent().data;
        s16 rightHindLegCurrent = sensor.GetLegCurrent().data;
        s16 leftHindLegCurrent = sensor.GetLegCurrent().data;
        s16 batteryCurrent = sensor.GetLegCurrent().data;
        EXPECT_EQ(rightFrontLegCurrent, motorCurrent1) << "num3 current1";
        EXPECT_EQ(leftFrontLegCurrent, motorCurrent2) << "num3 current2";
        EXPECT_EQ(rightHindLegCurrent, motorCurrent3) << "num3 current3";
        EXPECT_EQ(leftHindLegCurrent, motorCurrent4) << "num3 current4";
        EXPECT_EQ(batteryCurrent, bmsCurrent) << "num3  batteryCurrent";
        // 功能码03H
        // CurrentInfo rx3 = {};
        // bool success = sensor.GetCurrent(&rx3);
        // if (success == true) {
        //     // 处理获取到的数据
        //     EXPECT_EQ(rx3.current1, motorCurrent1) << "num3 current1";
        //     EXPECT_EQ(rx3.current2, motorCurrent2) << "num3 current2";
        //     EXPECT_EQ(rx3.current3, motorCurrent3) << "num3 current3";
        //     EXPECT_EQ(rx3.current4, motorCurrent4) << "num3 current4";
        //     EXPECT_EQ(rx3.batteryCurrent, bmsCurrent) << "num3  batteryCurrent";
        // } else {
        //     // 处理获取数据失败的情况
        //     // 处理获取到的数据
        //     EXPECT_EQ(1, 0xFF) << "num3 data err,i =" << i;
        // }
        readData.clear();
    }

    {
        // 功能码4无数据域
        // 添加功能码4，获取急停按键的信息
        CanTxCheckStopMotor();
        // readData.insert(readData.end(), {0xAA, 0x0A, 0x00, 0x00, 0x04, 0x01, 0x55, 0x55, 0x02, 0x66, 0x66, 0x00, 0x7D});
        TimerTools::SleepForMs(30);

        // 测试急停按键
        bool emergStop = sensor.GetEmergStop().data;
        EXPECT_EQ(emergStop, 0x01) << "num4";  // 收到急停则该数据必为1
        // 功能码04H
        // bool rx4 = {};
        // bool success = sensor.GetEmergStop(&rx4);
        // if (success == true) {
        //     // 处理获取到的数据
        //     EXPECT_EQ(rx4, 0x01) << "num4";  // 收到急停则该数据必为1
        // } else {
        //     // 处理获取数据失败的情况
        //     // 处理获取到的数据
        //     EXPECT_EQ(1, 0xFF) << "num4 data err";
        // }
        readData.clear();
    }

    u8 batteryLevel;
    s16 batteryCurrent;
    s16 batteryVoltage;
    u8 batteryStatus;
    for (size_t i = 0; i < TEST1NUM; i++) {
        if (i == 0) {
            batteryLevel = std::numeric_limits<decltype(batteryLevel)>::min();
            batteryCurrent = std::numeric_limits<decltype(batteryCurrent)>::min();
            batteryVoltage = std::numeric_limits<decltype(batteryVoltage)>::min();
            batteryStatus = std::numeric_limits<decltype(batteryStatus)>::min();
        } else if (i == 1) {
            batteryLevel = std::numeric_limits<decltype(batteryLevel)>::max();
            batteryCurrent = std::numeric_limits<decltype(batteryCurrent)>::max();
            batteryVoltage = std::numeric_limits<decltype(batteryVoltage)>::max();
            batteryStatus = std::numeric_limits<decltype(batteryStatus)>::max();

        } else if (i == 2) {
            batteryLevel = 0;
            batteryCurrent = 0;
            batteryVoltage = 0;
            batteryStatus = 0;
        } else {
            batteryLevel = std::rand() % (std::numeric_limits<decltype(batteryLevel)>::max());
            batteryCurrent = std::rand() % (std::numeric_limits<decltype(batteryCurrent)>::max());
            batteryVoltage = std::rand() % (std::numeric_limits<decltype(batteryVoltage)>::max());
            batteryStatus = std::rand() % (std::numeric_limits<decltype(batteryStatus)>::max());
        }
        // 添加功能码5，获取电池包的电量，电流，电压
        CanTxBatteryPower(batteryLevel, batteryCurrent, batteryVoltage, batteryStatus);
        TimerTools::SleepForMs(40);

        // 测试电池包参数，电量，电流，电压
        u16 smallQuantity = sensor.GetQuantity().data;
        u16 smallBMSCurrent = sensor.GetBC().data;
        u16 smallBMSVoltage = sensor.GetSmallBMSVoltage().data;
        bool BMSPowerStatus = std::get<1>(sensor.GetBMSPowerStatus()).state;
        EXPECT_EQ(smallQuantity, batteryLevel) << "num5 quantity err,i =" << i;
        EXPECT_EQ(smallBMSCurrent, batteryCurrent) << "num5 current err,i =" << i;
        EXPECT_EQ(smallBMSVoltage, batteryVoltage) << "num5 voltage err,batteryVoltage =" << batteryVoltage;
        EXPECT_EQ(BMSPowerStatus, batteryStatus) << "num5 voltage err,i =" << i;
        // 功能码05H
        // BatteryInfo rx5 = {};
        // bool success = sensor.GetBattery(&rx5);
        // if (success == true) {
        //     // 处理获取到的数据
        //     EXPECT_EQ(rx5.quantity, batteryLevel) << "num5 quantity err,i =" << i;
        //     EXPECT_EQ(rx5.voltage, batteryVoltage) << "num5 voltage err,batteryVoltage =" << batteryVoltage;
        //     EXPECT_EQ(rx5.charge, batteryStatus) << "num5 voltage err,i =" << i;
        //     EXPECT_EQ(rx5.current, batteryCurrent) << "num5 current err,i =" << i;
        // } else {
        //     // 处理获取数据失败的情况
        //     // 处理获取到的数据
        //     EXPECT_EQ(1, 0xFF) << "num5 data err,i =" << i;
        // }
        readData.clear();
    }

    u8 batteryTemperature1;
    u8 batteryTemperature2;
    u8 batteryTemperature3;
    u8 batteryTemperature4;
    u8 motorTempT3;
    u8 bmsTempT1;
    u8 bmsTempT2;
    u8 bmsTempT3;
    u8 errBit;
    for (size_t i = 0; i < TEST1NUM; i++) {
        if (i == 0) {
            batteryTemperature1 = std::numeric_limits<decltype(batteryTemperature1)>::min();
            batteryTemperature2 = std::numeric_limits<decltype(batteryTemperature2)>::min();
            batteryTemperature3 = std::numeric_limits<decltype(batteryTemperature3)>::min();
            batteryTemperature4 = std::numeric_limits<decltype(batteryTemperature4)>::min();

            motorTempT3 = std::numeric_limits<decltype(motorTempT3)>::min();
            bmsTempT1 = std::numeric_limits<decltype(bmsTempT1)>::min();
            bmsTempT2 = std::numeric_limits<decltype(bmsTempT2)>::min();
            bmsTempT2 = std::numeric_limits<decltype(bmsTempT3)>::min();

            errBit = std::numeric_limits<decltype(errBit)>::min();
        } else if (i == 1) {
            batteryTemperature1 = std::numeric_limits<decltype(batteryTemperature1)>::max();
            batteryTemperature2 = std::numeric_limits<decltype(batteryTemperature2)>::max();
            batteryTemperature3 = std::numeric_limits<decltype(batteryTemperature3)>::max();
            batteryTemperature4 = std::numeric_limits<decltype(batteryTemperature4)>::max();

            motorTempT3 = std::numeric_limits<decltype(motorTempT3)>::max();
            bmsTempT1 = std::numeric_limits<decltype(bmsTempT1)>::max();
            bmsTempT2 = std::numeric_limits<decltype(bmsTempT2)>::max();
            bmsTempT3 = std::numeric_limits<decltype(bmsTempT3)>::max();

            errBit = std::numeric_limits<decltype(errBit)>::max();
        } else if (i == 2) {
            batteryTemperature1 = 0;
            batteryTemperature2 = 0;
            batteryTemperature3 = 0;
            batteryTemperature4 = 0;

            motorTempT3 = 0;
            bmsTempT1 = 0;
            bmsTempT2 = 0;
            bmsTempT3 = 0;

            errBit = 0;
        } else {
            batteryTemperature1 = std::rand() % (std::numeric_limits<decltype(batteryTemperature1)>::max());
            batteryTemperature2 = std::rand() % (std::numeric_limits<decltype(batteryTemperature2)>::max());
            batteryTemperature3 = std::rand() % (std::numeric_limits<decltype(batteryTemperature3)>::max());
            batteryTemperature4 = std::rand() % (std::numeric_limits<decltype(batteryTemperature4)>::max());

            motorTempT3 = std::rand() % (std::numeric_limits<decltype(motorTempT3)>::max());
            bmsTempT1 = std::rand() % (std::numeric_limits<decltype(bmsTempT1)>::max());
            bmsTempT2 = std::rand() % (std::numeric_limits<decltype(bmsTempT2)>::max());
            bmsTempT3 = std::rand() % (std::numeric_limits<decltype(bmsTempT3)>::max());

            errBit = std::rand() % (std::numeric_limits<decltype(errBit)>::max());
        }
        // 添加功能码6,获取温度等
        CanTxPowerPcbTemperature1(batteryTemperature1, batteryTemperature2, batteryTemperature3, batteryTemperature4);
        CanTxPowerPcbTemperature2(motorTempT3);
        CanTxPowerPcbTemperature3(bmsTempT1, bmsTempT2, bmsTempT3, errBit);

        TimerTools::SleepForMs(30);

        // 测试电池包温度
        std::array<TimestampedData<s16>, 4> midBMSTemperatureArr = sensor.GetMidsizeBMSTemperature();                                     // 中型电池包温度数据数组
        std::array<TimestampedData<s16>, 3> smallBMSTemperatureArr = sensor.GetSmallBMSTemperature();                                     // 小型电池包温度数据数组
        s16 midPowerPcbTemperature = sensor.GetMidsizePowerPcbTemperature().data;                                                         // 中型电池包pcb板温度参数
        std::array<u8, 4> txMidBMSTemperatureArr = {batteryTemperature1, batteryTemperature2, batteryTemperature3, batteryTemperature4};  // 获取得到的中型电池包温度
        bmsTempT3 = 0;
        std::array<u8, 3> txsmallBMSTemperatureArr = {bmsTempT1, bmsTempT2, bmsTempT3};  // 获取得到的小型电池包温度
        for (size_t i = 0; i < 4; i++) {
            EXPECT_EQ(midBMSTemperatureArr[i].data, txMidBMSTemperatureArr[i]) << "num5 Temperature1 err, i =" << i;
        }
        for (size_t i = 0; i < 3; i++) {
            EXPECT_EQ(smallBMSTemperatureArr[i].data, txsmallBMSTemperatureArr[i]) << "num5 Temperature1 err, i =" << i;
        }
        EXPECT_EQ(midPowerPcbTemperature, motorTempT3) << "num5 powerPcbTemperature err,i =" << i;
        // 功能码06H
        // TemperatureInfo rx6 = {};
        // bool success = sensor.GetTemperature(&rx6);
        // if (success == true) {
        //     // 处理获取到的数据
        //     EXPECT_EQ(rx6.batteryTemperature1, batteryTemperature1) << "num5 Temperature1 err,i =" << i;
        //     EXPECT_EQ(rx6.batteryTemperature2, batteryTemperature2) << "num5 Temperature2 err,i =" << i;
        //     EXPECT_EQ(rx6.batteryTemperature3, batteryTemperature3) << "num5 Temperature3 err,i =" << i;
        //     EXPECT_EQ(rx6.batteryTemperature4, batteryTemperature4) << "num5 Temperature4 err,i =" << i;

        //     EXPECT_EQ(rx6.powerPcbTemperature, motorTempT3) << "num5 powerPcbTemperature err,i =" << i;

        //     EXPECT_EQ(rx6.bmsTemperature1, bmsTempT1) << "num5 Temperature2 err,i =" << i;
        //     EXPECT_EQ(rx6.bmsTemperature2, bmsTempT2) << "num5 Temperature3 err,i =" << i;
        //     EXPECT_EQ(rx6.bmsTemperature3, bmsTempT3) << "num5 powerPcbTemperature err,i =" << i;
        //     EXPECT_EQ(rx6.bmsTemperatureState, errBit) << "num5 powerPcbTemperature err,i =" << i;

        // } else {
        //     EXPECT_EQ(1, 0xFF) << "num6 data err,i =" << i;
        // }
        readData.clear();
    }
};

// 用于非定时上报的桩函数
static queue<array<u8, 13>> dataFifo;

bool Actively_Stub_Write(void *pThis, const std::vector<u8> &data)
{
    UNUSED(pThis);
    UNUSED(data);
    array<u8, 13> arr;
    // CanTxReplySlaveAddr();
    readData.clear();

    if (dataFifo.empty() == false) {
        readDataFlag = 1;
        arr = dataFifo.front();
        for (const auto &element : arr) {
            // 将元素添加到 readData 中
            readData.push_back(element);
        }
        dataFifo.pop();
    }
    return 1;
}

void ActivelyPutTestFifo(uint8_t TxData[8])
{
    uint8_t dataBuff[13] = {0};
    const uint8_t StdId = 1;
    uint8_t add = 0;
    dataBuff[0] = 0xAA;
    dataBuff[1] = 10;
    dataBuff[2] = StdId >> 8;
    dataBuff[3] = StdId;
    dataBuff[4] = TxData[0];
    dataBuff[5] = TxData[1];
    dataBuff[6] = TxData[2];
    dataBuff[7] = TxData[3];
    dataBuff[8] = TxData[4];
    dataBuff[9] = TxData[5];
    dataBuff[10] = TxData[6];
    dataBuff[11] = TxData[7];
    for (size_t i = 0; i < 10; i++) {
        add += dataBuff[2 + i];
    }
    dataBuff[12] = add;
    // 创建一个 array 对象
    array<u8, 13> arr;

    for (int i = 0; i < 13; i++) {
        arr[i] = dataBuff[i];
    }
    dataFifo.push(arr);
}
// 非定时上报（控制器下发询问），正常数据中的测试。测试需要耗时间5*2.1+5*600+5*600+5*600 = 9+10.5=19.5s
TEST_F(CanDeal_config_base, activelyBatteryInfo)
{
    while (!dataFifo.empty()) {
        dataFifo.pop();
    }

    // 一下两个桩用来模拟接受数据时序出队列的操作
    stub.set(PutTestFifo, ActivelyPutTestFifo);
    stub.set(ADDR(Uart, Write), Actively_Stub_Write);

    lock_guard<mutex> lock(rxMutex);
    readData.clear();
    HardwareSensorAPI sensor;
    {
        uint8_t slaveAddr;
        // 组包并存入队列
        for (size_t i = 0; i < TEST1NUM; i++) {
            if (i == 0) {
                slaveAddr = std::numeric_limits<uint8_t>::min();
            } else if (i == 1) {
                slaveAddr = std::numeric_limits<uint8_t>::max();
            } else if (i == 2) {
                slaveAddr = 0;
            } else {
                slaveAddr = std::rand() % (std::numeric_limits<uint8_t>::max());
            }
            CanTxReplySlaveAddr(slaveAddr);

            std::pair<bool, std::vector<BoardInfo>> rx0;
            rx0 = sensor.GetPCBNumber();
            EXPECT_EQ(rx0.first, true) << "num0 err,i =" << i;
            if (rx0.first == true) {
                EXPECT_EQ(rx0.second.back().canId, slaveAddr) << "num0 err,i =" << i;
            }
            readData.clear();
        }
    }

    {
        uint8_t PowerType = 1;
        uint8_t Powerindex = 1;
        bool PowerStara;
        for (size_t i = 0; i < TEST1NUM; i++) {
            if (i == 0) {
                // 现在API对于PowerType和Powerindex的值存在限制，故不对Powerindex和PowerType进行测试
                //  PowerType = std::numeric_limits<decltype(PowerType)>::min();
                //  Powerindex = std::numeric_limits<decltype(Powerindex)>::min();
                PowerStara = std::numeric_limits<bool>::min();
            } else if (i == 1) {
                // PowerType = std::numeric_limits<decltype(PowerType)>::max();
                // Powerindex = std::numeric_limits<decltype(Powerindex)>::max();
                PowerStara = std::numeric_limits<bool>::max();

            } else if (i == 2) {
                // PowerType = 0;
                // Powerindex = 0;
                PowerStara = 0;
            } else {
                // PowerType = std::rand() % (std::numeric_limits<decltype(PowerType)>::max());
                // Powerindex = std::rand() % (std::numeric_limits<decltype(Powerindex)>::max());
                PowerStara = std::rand() % (std::numeric_limits<bool>::max());
            }
            CanTxReplyStartMotorPower(PowerType, Powerindex, PowerStara);

            // 判断电机电源开关状态
            std::pair<bool, PowerInfo> motorPowerStatus = sensor.GetMotorPowerStatus();
            EXPECT_EQ(motorPowerStatus.second.kind, PowerType) << "num0 err,i =" << i;
            EXPECT_EQ(motorPowerStatus.second.index, Powerindex) << "num0 err,i =" << i;
            EXPECT_EQ(motorPowerStatus.second.state, PowerStara) << "num0 err,i =" << i;
            // std::pair<bool, std::vector<PowerInfo>> rx1;
            // rx1 = sensor.SetMotorPowerStatus(PowerType, Powerindex, PowerStara);
            // EXPECT_EQ(rx1.first, true) << "num0 err,i =" << i;
            // if (rx1.first == true) {
            //     EXPECT_EQ(rx1.second.back().kind, PowerType) << "num0 err,i =" << i;
            //     EXPECT_EQ(rx1.second.back().index, Powerindex) << "num0 err,i =" << i;
            //     EXPECT_EQ(rx1.second.back().state, PowerStara) << "num0 err,i =" << i;
            // }
            readData.clear();
        }
    }

    {
        uint8_t PowerTypeRead = 1;
        uint8_t PowerindexRead = 1;

        bool PowerStaraRead;
        for (size_t i = 0; i < TEST1NUM; i++) {
            if (i == 0) {
                // 现在API对于PowerType和Powerindex的值存在限制，故不对Powerindex和PowerType进行测试
                //  PowerTypeRead = std::numeric_limits<decltype(PowerTypeRead)>::min();
                //  PowerindexRead = std::numeric_limits<decltype(PowerindexRead)>::min();
                PowerStaraRead = std::numeric_limits<bool>::min();
            } else if (i == 1) {
                // PowerTypeRead = std::numeric_limits<decltype(PowerTypeRead)>::max();
                // PowerindexRead = std::numeric_limits<decltype(PowerindexRead)>::max();
                PowerStaraRead = std::numeric_limits<bool>::max();
            } else if (i == 2) {
                // PowerTypeRead = 0;
                // PowerindexRead = 0;
                PowerStaraRead = 0;
            } else {
                // PowerTypeRead = std::rand() % (std::numeric_limits<decltype(PowerTypeRead)>::max());
                // PowerindexRead = std::rand() % (std::numeric_limits<decltype(PowerindexRead)>::max());
                PowerStaraRead = std::rand() % (std::numeric_limits<bool>::max());
            }
            CanTxReplyMotorPowerState(PowerTypeRead, PowerindexRead, PowerStaraRead);
            // std::pair<bool, std::vector<PowerInfo>> rx2;
            // rx2 = sensor.BatteryrOffPower(PowerTypeRead, PowerindexRead);
            // EXPECT_EQ(rx2.first, true) << "num0 err,i =" << i;
            // if (rx2.first == true) {
            //     EXPECT_EQ(rx2.second.back().kind, PowerTypeRead) << "num0 err,i =" << i;
            //     EXPECT_EQ(rx2.second.back().index, PowerindexRead) << "num0 err,i =" << i;
            //     EXPECT_EQ(rx2.second.back().state, PowerStaraRead) << "num0 err,i =" << i;
            // }
            readData.clear();
        }
    }

    // {
    //     for (size_t i = 0; i < TEST1NUM; i++) {
    //         CanTxReplyOffThePower();

    //         std::pair<bool, bool> rx7;
    //         rx7 = sensor.TurnOffSystemPower();
    //         EXPECT_EQ(rx7.first, true) << "num0 err,i =" << i;
    //         readData.clear();
    //     }
    // }
};

/************************以下是错误代码的测试框架,其中定时上报只测试校验和错误的情况（错误测试测试1），非定时上报测试只测试
 * 回复超时（错误测试测试2）或者过期数据不接受（错误测试测试3）这两种情况*********************/

const uint8_t TEST1NUMErr = 1;

// 定时上报，非正常数据的测试（错误测试测试1）
TEST_F(CanDeal_config_base, errorInfo)
{
    readData.clear();
    // tag不对
    lock_guard<mutex> lock(rxMutex);
    stub.set(PutTestFifo, PutTestFifoErr);

    HardwareSensorAPI sensor;

    // 由于现在的can相关的代码使用的是单例，而IPC通信基于IP C所被调用的类，且暂时没有清空IPC内容的AIP，所以需要先在这里读取数据，清除正常数据发送所带来的残留数据
    // 如果之前IPC没有发送过正确信息则这里不需要读取数据
    // bool success;
    // {
    //     success = sensor.GetCurrent(&rx3);
    //     EXPECT_EQ(success, true) << "num3 true err";

    //     success = sensor.GetEmergStop(&rx4);
    //     EXPECT_EQ(success, true) << "num4 true err";

    //     success = sensor.GetBattery(&rx5);
    //     EXPECT_EQ(success, true) << "num5 true err";

    //     success = sensor.GetTemperature(&rx6);
    //     EXPECT_EQ(success, true) << "num6 true err";
    //     readData.clear();
    // }

    s16 motorCurrent1, motorCurrent2, motorCurrent3, motorCurrent4;
    for (size_t i = 0; i < TEST1NUMErr; i++) {
        s16 bmsCurrent;
        if (i == 0) {
            motorCurrent1 = std::numeric_limits<decltype(motorCurrent1)>::min();
            motorCurrent2 = std::numeric_limits<decltype(motorCurrent2)>::min();
            motorCurrent3 = std::numeric_limits<decltype(motorCurrent3)>::min();
            motorCurrent4 = std::numeric_limits<decltype(motorCurrent4)>::min();
            bmsCurrent = std::numeric_limits<decltype(bmsCurrent)>::min();
        } else if (i == 1) {
            motorCurrent1 = std::numeric_limits<decltype(motorCurrent1)>::max();
            motorCurrent2 = std::numeric_limits<decltype(motorCurrent2)>::max();
            motorCurrent3 = std::numeric_limits<decltype(motorCurrent3)>::max();
            motorCurrent4 = std::numeric_limits<decltype(motorCurrent4)>::max();
            bmsCurrent = std::numeric_limits<decltype(bmsCurrent)>::max();
        } else if (i == 2) {
            motorCurrent1 = 0;
            motorCurrent2 = 0;
            motorCurrent3 = 0;
            motorCurrent4 = 0;
            bmsCurrent = 0;
        } else {
            motorCurrent1 = std::rand() % (std::numeric_limits<decltype(motorCurrent4)>::max());
            motorCurrent2 = std::rand() % (std::numeric_limits<decltype(motorCurrent4)>::max());
            motorCurrent3 = std::rand() % (std::numeric_limits<decltype(motorCurrent4)>::max());
            motorCurrent4 = std::rand() % (std::numeric_limits<decltype(motorCurrent4)>::max());
            bmsCurrent = std::rand() % (std::numeric_limits<decltype(motorCurrent4)>::max());
        }
        CanTxmotorCurrent(motorCurrent1, motorCurrent2, motorCurrent3, motorCurrent4);  // 索引1234
        CanTxBatteryPackCurrent(bmsCurrent);                                            // 索引5

        TimerTools::SleepForMs(25);
        // 功能码03H
        // bool success = sensor.GetCurrent(&rx3);
        // EXPECT_EQ(success, false) << "num3EErr,i =" << i;
        readData.clear();
    }

    {
        // 功能码4无数据域
        // 添加功能码4，获取急停按键的信息
        CanTxCheckStopMotor();
        // readData.insert(readData.end(), {0xAA, 0x0A, 0x00, 0x00, 0x04, 0x01, 0x55, 0x55, 0x02, 0x66, 0x66, 0x00, 0x7D});
        TimerTools::SleepForMs(30);
        // 功能码04H
        // bool rx4 = {};
        // bool success = sensor.GetEmergStop(&rx4);
        // EXPECT_EQ(success, false) << "num4Err";
        readData.clear();
    }

    u8 batteryLevel;
    s16 batteryCurrent;
    s16 batteryVoltage;
    u8 batteryStatus;
    for (size_t i = 0; i < TEST1NUMErr; i++) {
        if (i == 0) {
            batteryLevel = std::numeric_limits<decltype(batteryLevel)>::min();
            batteryCurrent = std::numeric_limits<decltype(batteryCurrent)>::min();
            batteryVoltage = std::numeric_limits<decltype(batteryVoltage)>::min();
            batteryStatus = std::numeric_limits<decltype(batteryStatus)>::min();
        } else if (i == 1) {
            batteryLevel = std::numeric_limits<decltype(batteryLevel)>::max();
            batteryCurrent = std::numeric_limits<decltype(batteryCurrent)>::max();
            batteryVoltage = std::numeric_limits<decltype(batteryVoltage)>::max();
            batteryStatus = std::numeric_limits<decltype(batteryStatus)>::max();

        } else if (i == 2) {
            batteryLevel = 0;
            batteryCurrent = 0;
            batteryVoltage = 0;
            batteryStatus = 0;
        } else {
            batteryLevel = std::rand() % (std::numeric_limits<decltype(batteryLevel)>::max());
            batteryCurrent = std::rand() % (std::numeric_limits<decltype(batteryCurrent)>::max());
            batteryVoltage = std::rand() % (std::numeric_limits<decltype(batteryVoltage)>::max());
            batteryStatus = std::rand() % (std::numeric_limits<decltype(batteryStatus)>::max());
        }
        // 添加功能码5，获取电池包的电量，电流，电压
        CanTxBatteryPower(batteryLevel, batteryCurrent, batteryVoltage, batteryStatus);

        TimerTools::SleepForMs(40);
        // 功能码05H
        // BatteryInfo rx5 = {};
        // bool success = sensor.GetBattery(&rx5);
        // EXPECT_EQ(success, false) << "num5Err,i =" << i;
        readData.clear();
    }
    u8 batteryTemperature1;
    u8 batteryTemperature2;
    u8 batteryTemperature3;
    u8 batteryTemperature4;
    u8 motorTempT3;
    u8 bmsTempT1;
    u8 bmsTempT2;
    u8 bmsTempT3 = 0;
    u8 errBit;
    for (size_t i = 0; i < TEST1NUMErr; i++) {
        if (i == 0) {
            batteryTemperature1 = std::numeric_limits<decltype(batteryTemperature1)>::min();
            batteryTemperature2 = std::numeric_limits<decltype(batteryTemperature2)>::min();
            batteryTemperature3 = std::numeric_limits<decltype(batteryTemperature3)>::min();
            batteryTemperature4 = std::numeric_limits<decltype(batteryTemperature4)>::min();

            motorTempT3 = std::numeric_limits<decltype(motorTempT3)>::min();
            bmsTempT1 = std::numeric_limits<decltype(bmsTempT1)>::min();
            bmsTempT2 = std::numeric_limits<decltype(bmsTempT2)>::min();
            bmsTempT2 = std::numeric_limits<decltype(bmsTempT3)>::min();

            errBit = std::numeric_limits<decltype(errBit)>::min();
        } else if (i == 1) {
            batteryTemperature1 = std::numeric_limits<decltype(batteryTemperature1)>::max();
            batteryTemperature2 = std::numeric_limits<decltype(batteryTemperature2)>::max();
            batteryTemperature3 = std::numeric_limits<decltype(batteryTemperature3)>::max();
            batteryTemperature4 = std::numeric_limits<decltype(batteryTemperature4)>::max();

            motorTempT3 = std::numeric_limits<decltype(motorTempT3)>::max();
            bmsTempT1 = std::numeric_limits<decltype(bmsTempT1)>::max();
            bmsTempT2 = std::numeric_limits<decltype(bmsTempT2)>::max();
            bmsTempT3 = std::numeric_limits<decltype(bmsTempT3)>::max();

            errBit = std::numeric_limits<decltype(errBit)>::max();
        } else if (i == 2) {
            batteryTemperature1 = 0;
            batteryTemperature2 = 0;
            batteryTemperature3 = 0;
            batteryTemperature4 = 0;

            motorTempT3 = 0;
            bmsTempT1 = 0;
            bmsTempT2 = 0;
            bmsTempT3 = 0;

            errBit = 0;
        } else {
            batteryTemperature1 = std::rand() % (std::numeric_limits<decltype(batteryTemperature1)>::max());
            batteryTemperature2 = std::rand() % (std::numeric_limits<decltype(batteryTemperature2)>::max());
            batteryTemperature3 = std::rand() % (std::numeric_limits<decltype(batteryTemperature3)>::max());
            batteryTemperature4 = std::rand() % (std::numeric_limits<decltype(batteryTemperature4)>::max());

            motorTempT3 = std::rand() % (std::numeric_limits<decltype(motorTempT3)>::max());
            bmsTempT1 = std::rand() % (std::numeric_limits<decltype(bmsTempT1)>::max());
            bmsTempT2 = std::rand() % (std::numeric_limits<decltype(bmsTempT2)>::max());
            bmsTempT3 = std::rand() % (std::numeric_limits<decltype(bmsTempT3)>::max());

            errBit = std::rand() % (std::numeric_limits<decltype(errBit)>::max());
        }
        // 添加功能码6,获取温度等
        CanTxPowerPcbTemperature1(batteryTemperature1, batteryTemperature2, batteryTemperature3, batteryTemperature4);  // 索引1
        CanTxPowerPcbTemperature2(motorTempT3);                                                                         // 索引2
        CanTxPowerPcbTemperature3(bmsTempT1, bmsTempT2, bmsTempT3, errBit);                                             // 索引3

        TimerTools::SleepForMs(30);
        // 功能码06H
        // bool success = sensor.GetTemperature(&rx6);
        // EXPECT_EQ(success, false) << "num4Err,i =" << i;
        readData.clear();
    }
};

// 非定时上报（控制器下发询问），（错误测试测试2）
TEST_F(CanDeal_config_base, activelyBatteryInfoErr1)
{
    // 一下两个桩用来模拟接受数据时序出队列的操作
    stub.set(PutTestFifo, ActivelyPutTestFifo);
    stub.set(ADDR(Uart, Write), Actively_Stub_Write);

    lock_guard<mutex> lock(rxMutex);
    readData.clear();
    HardwareSensorAPI sensor;
    {
        uint8_t slaveAddr;
        // 组包并存入队列
        for (size_t i = 0; i < TEST1NUM; i++) {
            while (!dataFifo.empty()) {
                dataFifo.pop();
            }
            if (i == 0) {
                slaveAddr = std::numeric_limits<uint8_t>::min();
            } else if (i == 1) {
                slaveAddr = std::numeric_limits<uint8_t>::max();
            } else if (i == 2) {
                slaveAddr = 0;
            } else {
                slaveAddr = std::rand() % (std::numeric_limits<uint8_t>::max());
            }

            std::pair<bool, std::vector<BoardInfo>> rx0;
            rx0 = sensor.GetPCBNumber();

            CanTxReplySlaveAddr(slaveAddr);

            EXPECT_EQ(rx0.first, false) << "num0 err,i =" << i;
            if (rx0.first == true) {
                EXPECT_EQ(rx0.second.back().canId, slaveAddr) << "num0 err,i =" << i;
            }
            readData.clear();
        }
    }
    {
        uint8_t PowerType = 1;
        uint8_t Powerindex = 1;
        bool PowerStara;
        for (size_t i = 0; i < TEST1NUM; i++) {
            while (!dataFifo.empty()) {
                dataFifo.pop();
            }
            if (i == 0) {
                // 现在API对于PowerType和Powerindex的值存在限制，故不对Powerindex和PowerType进行测试
                //  PowerType = std::numeric_limits<decltype(PowerType)>::min();
                //  Powerindex = std::numeric_limits<decltype(Powerindex)>::min();
                PowerStara = std::numeric_limits<bool>::min();
            } else if (i == 1) {
                // PowerType = std::numeric_limits<decltype(PowerType)>::max();
                // Powerindex = std::numeric_limits<decltype(Powerindex)>::max();
                PowerStara = std::numeric_limits<bool>::max();

            } else if (i == 2) {
                // PowerType = 0;
                // Powerindex = 0;
                PowerStara = 0;
            } else {
                // PowerType = std::rand() % (std::numeric_limits<decltype(PowerType)>::max());
                // Powerindex = std::rand() % (std::numeric_limits<decltype(Powerindex)>::max());
                PowerStara = std::rand() % (std::numeric_limits<bool>::max());
            }
            CanTxReplyStartMotorPower(PowerType, Powerindex, PowerStara);

            // 判断电机电源开关状态
            std::pair<bool, PowerInfo> motorPowerStatus = sensor.GetMotorPowerStatus();
            EXPECT_EQ(motorPowerStatus.second.kind, PowerType) << "num0 err,i =" << i;
            EXPECT_EQ(motorPowerStatus.second.index, Powerindex) << "num0 err,i =" << i;
            EXPECT_EQ(motorPowerStatus.second.state, PowerStara) << "num0 err,i =" << i;
            // std::pair<bool, std::vector<PowerInfo>> rx1;
            // rx1 = sensor.SetMotorPowerStatus(PowerType, Powerindex, PowerStara);
            // EXPECT_EQ(rx1.first, false) << "num0 err,i =" << i;
            // if (rx1.first == true) {
            //     EXPECT_EQ(rx1.second.back().kind, PowerType) << "num0 err,i =" << i;
            //     EXPECT_EQ(rx1.second.back().index, Powerindex) << "num0 err,i =" << i;
            //     EXPECT_EQ(rx1.second.back().state, PowerStara) << "num0 err,i =" << i;
            // }
            readData.clear();
        }
    }
    {
        // uint8_t PowerTypeRead = 1;
        // uint8_t PowerindexRead = 1;
        bool PowerStaraRead;
        for (size_t i = 0; i < TEST1NUM; i++) {
            while (!dataFifo.empty()) {
                dataFifo.pop();
            }
            if (i == 0) {
                // 现在API对于PowerType和Powerindex的值存在限制，故不对Powerindex和PowerType进行测试
                //  PowerTypeRead = std::numeric_limits<decltype(PowerTypeRead)>::min();
                //  PowerindexRead = std::numeric_limits<decltype(PowerindexRead)>::min();
                PowerStaraRead = std::numeric_limits<bool>::min();
            } else if (i == 1) {
                // PowerTypeRead = std::numeric_limits<decltype(PowerTypeRead)>::max();
                // PowerindexRead = std::numeric_limits<decltype(PowerindexRead)>::max();
                PowerStaraRead = std::numeric_limits<bool>::max();
            } else if (i == 2) {
                // PowerTypeRead = 0;
                // PowerindexRead = 0;
                PowerStaraRead = 0;
            } else {
                // PowerTypeRead = std::rand() % (std::numeric_limits<decltype(PowerTypeRead)>::max());
                // PowerindexRead = std::rand() % (std::numeric_limits<decltype(PowerindexRead)>::max());
                PowerStaraRead = std::rand() % (std::numeric_limits<bool>::max());
            }
            (void)PowerStaraRead;
            // std::pair<bool, std::vector<PowerInfo>> rx2;
            // rx2 = sensor.BatteryrOffPower(PowerTypeRead, PowerindexRead);
            // CanTxReplyMotorPowerState(PowerTypeRead, PowerindexRead, PowerStaraRead);
            // EXPECT_EQ(rx2.first, false) << "num0 err,i =" << i;
            // if (rx2.first == true) {
            //     EXPECT_EQ(rx2.second.back().kind, PowerTypeRead) << "num0 err,i =" << i;
            //     EXPECT_EQ(rx2.second.back().index, PowerindexRead) << "num0 err,i =" << i;
            //     EXPECT_EQ(rx2.second.back().state, PowerStaraRead) << "num0 err,i =" << i;
            // }
            readData.clear();
        }
    }

    {
        for (size_t i = 0; i < TEST1NUM; i++) {
            while (!dataFifo.empty()) {
                dataFifo.pop();
            }
            // std::pair<bool, bool> rx7;
            // rx7 = sensor.TurnOffSystemPower();
            // EXPECT_EQ(rx7.first, false) << "num0 err,i =" << i;
            // CanTxReplyOffThePower();
            readData.clear();
        }
    }
};

// 非定时上报（控制器下发询问），测试数据时效性，在下发询问之前回复的的数据不能被接收（错误测试测试3）
TEST_F(CanDeal_config_base, activelyBatteryInfoErr)
{
    lock_guard<mutex> lock(rxMutex);
    readData.clear();
    HardwareSensorAPI sensor;

    {
        uint8_t slaveAddr;
        // 组包并存入队列
        for (size_t i = 0; i < TEST1NUM; i++) {
            if (i == 0) {
                slaveAddr = std::numeric_limits<uint8_t>::min();
            } else if (i == 1) {
                slaveAddr = std::numeric_limits<uint8_t>::max();
            } else if (i == 2) {
                slaveAddr = 0;
            } else {
                slaveAddr = std::rand() % (std::numeric_limits<uint8_t>::max());
            }
            CanTxReplySlaveAddr(slaveAddr);

            TimerTools::SleepForMs(30);

            std::pair<bool, std::vector<BoardInfo>> rx0;
            rx0 = sensor.GetPCBNumber();
            EXPECT_EQ(rx0.first, false) << "num0 err,i =" << i;
            if (rx0.first == true) {
                EXPECT_EQ(rx0.second.back().canId, slaveAddr) << "num0 err,i =" << i;
            }
            readData.clear();
        }
    }
    {
        uint8_t PowerType = 1;
        uint8_t Powerindex = 1;
        bool PowerStara;
        for (size_t i = 0; i < TEST1NUM; i++) {
            if (i == 0) {
                // 现在API对于PowerType和Powerindex的值存在限制，故不对Powerindex和PowerType进行测试
                //  PowerType = std::numeric_limits<decltype(PowerType)>::min();
                //  Powerindex = std::numeric_limits<decltype(Powerindex)>::min();
                PowerStara = std::numeric_limits<bool>::min();
            } else if (i == 1) {
                // PowerType = std::numeric_limits<decltype(PowerType)>::max();
                // Powerindex = std::numeric_limits<decltype(Powerindex)>::max();
                PowerStara = std::numeric_limits<bool>::max();

            } else if (i == 2) {
                // PowerType = 0;
                // Powerindex = 0;
                PowerStara = 0;
            } else {
                // PowerType = std::rand() % (std::numeric_limits<decltype(PowerType)>::max());
                // Powerindex = std::rand() % (std::numeric_limits<decltype(Powerindex)>::max());
                PowerStara = std::rand() % (std::numeric_limits<bool>::max());
            }
            CanTxReplyStartMotorPower(PowerType, Powerindex, PowerStara);
            TimerTools::SleepForMs(30);

            // 判断电机电源开关状态
            std::pair<bool, PowerInfo> motorPowerStatus = sensor.GetMotorPowerStatus();
            EXPECT_EQ(motorPowerStatus.second.kind, PowerType) << "num0 err,i =" << i;
            EXPECT_EQ(motorPowerStatus.second.index, Powerindex) << "num0 err,i =" << i;
            EXPECT_EQ(motorPowerStatus.second.state, PowerStara) << "num0 err,i =" << i;
            // std::pair<bool, std::vector<PowerInfo>> rx1;
            // rx1 = sensor.SetMotorPowerStatus(PowerType, Powerindex, PowerStara);
            // EXPECT_EQ(rx1.first, false) << "num0 err,i =" << i;
            // if (rx1.first == true) {
            //     EXPECT_EQ(rx1.second.back().kind, PowerType) << "num0 err,i =" << i;
            //     EXPECT_EQ(rx1.second.back().index, Powerindex) << "num0 err,i =" << i;
            //     EXPECT_EQ(rx1.second.back().state, PowerStara) << "num0 err,i =" << i;
            // }
            readData.clear();
        }
    }
    {
        uint8_t PowerTypeRead = 1;
        uint8_t PowerindexRead = 1;

        bool PowerStaraRead;
        for (size_t i = 0; i < TEST1NUM; i++) {
            if (i == 0) {
                // 现在API对于PowerType和Powerindex的值存在限制，故不对Powerindex和PowerType进行测试
                //  PowerTypeRead = std::numeric_limits<decltype(PowerTypeRead)>::min();
                //  PowerindexRead = std::numeric_limits<decltype(PowerindexRead)>::min();
                PowerStaraRead = std::numeric_limits<bool>::min();
            } else if (i == 1) {
                // PowerTypeRead = std::numeric_limits<decltype(PowerTypeRead)>::max();
                // PowerindexRead = std::numeric_limits<decltype(PowerindexRead)>::max();
                PowerStaraRead = std::numeric_limits<bool>::max();
            } else if (i == 2) {
                // PowerTypeRead = 0;
                // PowerindexRead = 0;
                PowerStaraRead = 0;
            } else {
                // PowerTypeRead = std::rand() % (std::numeric_limits<decltype(PowerTypeRead)>::max());
                // PowerindexRead = std::rand() % (std::numeric_limits<decltype(PowerindexRead)>::max());
                PowerStaraRead = std::rand() % (std::numeric_limits<bool>::max());
            }
            CanTxReplyMotorPowerState(PowerTypeRead, PowerindexRead, PowerStaraRead);
            TimerTools::SleepForMs(30);

            // std::pair<bool, std::vector<PowerInfo>> rx2;
            // rx2 = sensor.BatteryrOffPower(PowerTypeRead, PowerindexRead);
            // EXPECT_EQ(rx2.first, false) << "num0 err,i =" << i;
            // if (rx2.first == true) {
            //     EXPECT_EQ(rx2.second.back().kind, PowerTypeRead) << "num0 err,i =" << i;
            //     EXPECT_EQ(rx2.second.back().index, PowerindexRead) << "num0 err,i =" << i;
            //     EXPECT_EQ(rx2.second.back().state, PowerStaraRead) << "num0 err,i =" << i;
            // }
            readData.clear();
        }
    }

    {
        for (size_t i = 0; i < TEST1NUM; i++) {
            CanTxReplyOffThePower();
            TimerTools::SleepForMs(30);

            // std::pair<bool, bool> rx7;
            // rx7 = sensor.TurnOffSystemPower();
            // EXPECT_EQ(rx7.first, true) << "num0 err,i =" << i;
            readData.clear();
        }
    }
};

// ./build/x64/output/qr_test

#else

uint8_t readDataFlag = 0;
static vector<u8> readData;
std::mutex rxMutex;
static queue<array<u8, 13>> dataFifo;
#define TEST1NUM 1

// 串口类接口的桩函数
bool stub_Open(void *pThis, const string &str)
{
    UNUSED(pThis);
    UNUSED(str);
    return true;
}
bool stub_SetParam(void *pThis, int nSpeed, int nBits, char nEvent, int nStop)
{
    UNUSED(pThis);
    UNUSED(nSpeed);
    UNUSED(nBits);
    UNUSED(nEvent);
    UNUSED(nStop);
    return true;
}
bool stub_SetBlockCnt(void *pThis, int cnt)
{
    UNUSED(pThis);
    UNUSED(cnt);
    return true;
}
void stub_Close(void *pThis) { UNUSED(pThis); }

bool stub_Write(void *pThis, const std::vector<u8> &data)
{
    UNUSED(pThis);
    UNUSED(data);

    return 1;
}

std::vector<u8> stub_Read(void *pThis, int ms = 2)
{
    UNUSED(pThis);
    UNUSED(ms);
    vector<u8> a;
    if (readDataFlag == 1) {
        readDataFlag = 0;
        return readData;

    } else {
        return a;
    }
}
void ActivelyPutTestFifo(uint8_t TxData[8])
{
    uint8_t dataBuff[13] = {0};
    const uint8_t StdId = 1;
    uint8_t add = 0;
    dataBuff[0] = 0xAA;
    dataBuff[1] = 10;
    dataBuff[2] = StdId >> 8;
    dataBuff[3] = StdId;
    dataBuff[4] = TxData[0];
    dataBuff[5] = TxData[1];
    dataBuff[6] = TxData[2];
    dataBuff[7] = TxData[3];
    dataBuff[8] = TxData[4];
    dataBuff[9] = TxData[5];
    dataBuff[10] = TxData[6];
    dataBuff[11] = TxData[7];
    for (size_t i = 0; i < 10; i++) {
        add += dataBuff[2 + i];
    }
    dataBuff[12] = add;
    // 创建一个 array 对象
    array<u8, 13> arr;

    for (int i = 0; i < 13; i++) {
        arr[i] = dataBuff[i];
    }
    dataFifo.push(arr);
}
/**
 * @brief 提供错误的校验和的函数
 *
 * @param TxData
 */
void PutTestFifoErr(uint8_t TxData[8])
{
    uint8_t dataBuff[13] = {0};
    const uint8_t StdId = 1;
    uint8_t add = 0;
    dataBuff[0] = 0xAA;
    dataBuff[1] = 10;
    dataBuff[2] = StdId >> 8;
    dataBuff[3] = StdId;
    dataBuff[4] = TxData[0];
    dataBuff[5] = TxData[1];
    dataBuff[6] = TxData[2];
    dataBuff[7] = TxData[3];
    dataBuff[8] = TxData[4];
    dataBuff[9] = TxData[5];
    dataBuff[10] = TxData[6];
    dataBuff[11] = TxData[7];
    for (size_t i = 0; i < 10; i++) {
        add += dataBuff[2 + i];
    }
    dataBuff[12] = (add + 1);
    readDataFlag = 1;
    readData.insert(readData.end(), std::begin(dataBuff), std::end(dataBuff));
}
void PutTestFifo(uint8_t TxData[8])
{
    uint8_t dataBuff[13] = {0};
    const uint8_t StdId = 1;
    uint8_t add = 0;
    dataBuff[0] = 0xAA;
    dataBuff[1] = 10;
    dataBuff[2] = StdId >> 8;
    dataBuff[3] = StdId;
    dataBuff[4] = TxData[0];
    dataBuff[5] = TxData[1];
    dataBuff[6] = TxData[2];
    dataBuff[7] = TxData[3];
    dataBuff[8] = TxData[4];
    dataBuff[9] = TxData[5];
    dataBuff[10] = TxData[6];
    dataBuff[11] = TxData[7];
    for (size_t i = 0; i < 10; i++) {
        add += dataBuff[2 + i];
    }
    dataBuff[12] = add;
    readDataFlag = 1;
    readData.insert(readData.end(), std::begin(dataBuff), std::end(dataBuff));
}

/* 以下为定时上报的协议，定时上报相关处理在实际运行中为数据上报之后数据自动存储，等待被应用层读取，故发数据组包的入栈可以在应用层调用读取AIP之前执行*/
/*电流检测（功能码03H），索引1,2,3,4.*/
void CanTxmotorCurrent(s16 motorCurrent1, s16 motorCurrent2, s16 motorCurrent3, s16 motorCurrent4)
{
    uint8_t TxData[8];
    // 电机电流的第一个报文
    TxData[0] = 0x03;                  // 功能码
    TxData[1] = 0x01;                  // 数据域，索引
    TxData[2] = (motorCurrent1) >> 8;  // 数据域，一个s16的值
    TxData[3] = motorCurrent1;         // 数据域，一个s16的值
    TxData[4] = 0x02;                  // 一个字节的索引
    TxData[5] = (motorCurrent2) >> 8;  // 数据域，一个s16的值
    TxData[6] = motorCurrent2;         // 数据域，一个s16的值
    TxData[7] = 0;                     // 保留
    PutTestFifo(TxData);

    // 电机电流的第二个报文
    TxData[0] = 0x03;                  // 功能码
    TxData[1] = 0x03;                  // 数据域，索引
    TxData[2] = (motorCurrent3) >> 8;  // 数据域，一个s16的值
    TxData[3] = motorCurrent3;         // 数据域，一个s16的值
    TxData[4] = 0x04;                  // 数据域，索引
    TxData[5] = (motorCurrent4) >> 8;  // 数据域，一个s16的值
    TxData[6] = motorCurrent4;         // 数据域，一个s16的值
    TxData[7] = 0;                     // 保留
    PutTestFifo(TxData);
}

/*电流检测（功能码03H），索引5。*/
void CanTxBatteryPackCurrent(s16 bmsCurrent)
{
    uint8_t TxData[8];

    TxData[0] = 0x03;               // 功能码
    TxData[1] = 0x05;               // 数据域，索引
    TxData[2] = (bmsCurrent) >> 8;  // 数据域，一个s16的值
    TxData[3] = bmsCurrent;         // 数据域，一个s16的值
    TxData[4] = 0x00;               // 保留
    TxData[5] = 0x00;               // 保留
    TxData[6] = 0x00;               // 保留
    TxData[7] = 0x00;               // 保留
    PutTestFifo(TxData);
}
/**正确数据的夹具*/
// 构造函数和析构函数，本类构造和析构时执行一次
// SetUp/TearDown函数，每个TEST_F都会执行一次
class CanDeal_config_base : public ::testing::Test
{
protected:
    CanDeal_config_base()
    {
        RobotLogSetOutput(LogPosi::none);
        GetRobotCfgFile().ParseFile("./config/00-base.toml");
        // 对底层函数打桩
        // ，stub.set方法用于替换函数的实现。它接受两个参数：第一个参数是要替换的函数的地址，第二个参数是替换后的函数的地址
        stub.set(ADDR(Uart, Open), stub_Open);
        stub.set(ADDR(Uart, SetParam), stub_SetParam);
        stub.set(ADDR(Uart, SetBlockCnt), stub_SetBlockCnt);
        stub.set(ADDR(Uart, Close), stub_Close);
        stub.set(ADDR(Uart, Write), stub_Write);
        stub.set(ADDR(Uart, Read), stub_Read);
    }
    ~CanDeal_config_base() override {}
    void SetUp() override
    {
        lock_guard<mutex> lock(rxMutex);
        {
            // 正常电池数据
            {
                // 定时上报
            }
        }
        GetCanBridge().Init();
        // can线程10ms运行一次，延时保证它一定执行过，若小于两倍可能导致canBridge未收到canDeal信息
        TimerTools::SleepForMs(30);
    }
    void TearDown() override {}

private:
    Stub stub;
};

s16 motorCurrent1, motorCurrent2, motorCurrent3, motorCurrent4;
s16 bmsCurrent;
// 定时上报（控制器下发询问），正常数据中的测试
TEST_F(CanDeal_config_base, activelyBatteryInfo_00)
{
    lock_guard<mutex> lock(rxMutex);
    HardwareSensorAPI sensor;

    for (size_t i = 0; i < TEST1NUM; i++) {
        if (i == 0) {
            motorCurrent1 = std::numeric_limits<decltype(motorCurrent1)>::min();
            motorCurrent2 = std::numeric_limits<decltype(motorCurrent2)>::min();
            motorCurrent3 = std::numeric_limits<decltype(motorCurrent3)>::min();
            motorCurrent4 = std::numeric_limits<decltype(motorCurrent4)>::min();
            bmsCurrent = std::numeric_limits<decltype(bmsCurrent)>::min();
        } else if (i == 1) {
            motorCurrent1 = std::numeric_limits<decltype(motorCurrent1)>::max();
            motorCurrent2 = std::numeric_limits<decltype(motorCurrent2)>::max();
            motorCurrent3 = std::numeric_limits<decltype(motorCurrent3)>::max();
            motorCurrent4 = std::numeric_limits<decltype(motorCurrent4)>::max();
            bmsCurrent = std::numeric_limits<decltype(bmsCurrent)>::max();
        } else if (i == 2) {
            motorCurrent1 = 0;
            motorCurrent2 = 0;
            motorCurrent3 = 0;
            motorCurrent4 = 0;
            bmsCurrent = 0;
        } else {
            motorCurrent1 = std::rand() % (std::numeric_limits<decltype(motorCurrent4)>::max());
            motorCurrent2 = std::rand() % (std::numeric_limits<decltype(motorCurrent4)>::max());
            motorCurrent3 = std::rand() % (std::numeric_limits<decltype(motorCurrent4)>::max());
            motorCurrent4 = std::rand() % (std::numeric_limits<decltype(motorCurrent4)>::max());
            bmsCurrent = std::rand() % (std::numeric_limits<decltype(motorCurrent4)>::max());
        }
        CanTxmotorCurrent(motorCurrent1, motorCurrent2, motorCurrent3, motorCurrent4);
        CanTxBatteryPackCurrent(bmsCurrent);
        TimerTools::SleepForMs(25);
        // 功能码03H
        CurrentInfo rx3 = {};
        bool success = sensor.GetCurrent(&rx3);
        if (success == true) {
            // 处理获取到的数据
            EXPECT_EQ(rx3.current1, motorCurrent1) << "num3 current1";
            EXPECT_EQ(rx3.current2, motorCurrent2) << "num3 current2";
            EXPECT_EQ(rx3.current3, motorCurrent3) << "num3 current3";
            EXPECT_EQ(rx3.current4, motorCurrent4) << "num3 current4";
            EXPECT_EQ(rx3.batteryCurrent, bmsCurrent) << "num3  batteryCurrent";
        } else {
            // 处理获取数据失败的情况
            // 处理获取到的数据
            EXPECT_EQ(1, 0xFF) << "num3 data err,i =" << i;
        }
        readData.clear();
    }
    {
        HardwareSensorAPI sensor;
        CurrentInfo rx3 = {};
        bool success = sensor.GetCurrent(&rx3);
        if (success == true) {
            // 处理获取到的数据
            EXPECT_EQ(rx3.current1, motorCurrent1) << "num3 current1";
            EXPECT_EQ(rx3.current2, motorCurrent2) << "num3 current2";
            EXPECT_EQ(rx3.current3, motorCurrent3) << "num3 current3";
            EXPECT_EQ(rx3.current4, motorCurrent4) << "num3 current4";
            EXPECT_EQ(rx3.batteryCurrent, bmsCurrent) << "num3  batteryCurrent";
        } else {
            // 处理获取数据失败的情况
            // 处理获取到的数据
            EXPECT_EQ(1, 0xFF) << "num3 data err,i =";
        }
    }
}

// 定时上报（控制器下发询问），正常数据中的测试
TEST_F(CanDeal_config_base, activelyBatteryInfo_01)
{
    readData.clear();
    HardwareSensorAPI sensor;
    CurrentInfo rx3 = {};
    bool success = sensor.GetCurrent(&rx3);
    EXPECT_EQ(success, false) << "num3EErr,i =";

    if (success == true) {
        // 处理获取到的数据
        EXPECT_EQ(rx3.current1, motorCurrent1) << "num3 current1";
        EXPECT_EQ(rx3.current2, motorCurrent2) << "num3 current2";
        EXPECT_EQ(rx3.current3, motorCurrent3) << "num3 current3";
        EXPECT_EQ(rx3.current4, motorCurrent4) << "num3 current4";
        EXPECT_EQ(rx3.batteryCurrent, bmsCurrent) << "num3  batteryCurrent";
    } else {
        // 处理获取数据失败的情况
        // 处理获取到的数据
        EXPECT_EQ(1, 0xFF) << "num3 data err,i =";
    }
    success = sensor.GetCurrent(&rx3);
    EXPECT_EQ(success, false) << "num3EErr,i =";

    if (success == true) {
        // 处理获取到的数据
        EXPECT_EQ(rx3.current1, motorCurrent1) << "num3 current1";
        EXPECT_EQ(rx3.current2, motorCurrent2) << "num3 current2";
        EXPECT_EQ(rx3.current3, motorCurrent3) << "num3 current3";
        EXPECT_EQ(rx3.current4, motorCurrent4) << "num3 current4";
        EXPECT_EQ(rx3.batteryCurrent, bmsCurrent) << "num3  batteryCurrent";
    } else {
        // 处理获取数据失败的情况
        // 处理获取到的数据
        EXPECT_EQ(1, 0xFF) << "num3 data err,i =";
    }
}

#endif

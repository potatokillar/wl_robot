
#pragma once  // #program once 参数　　其作用是在在头文件的最开始加入这条指令，以保证头文件被编译一次。但#program
              // once是编译器相关的，就是说在这个编译系统上能用，但在其他的编译系统上就不一定能用，所以其可移植性较差。一般如果强调程序跨平台，还是选择使用“#ifndef,   #define,
              // #endif”比较好。
#include <thread>

#include "baseline.hpp"
#include "canDeal.hpp"
#include "sensorType.hpp"

// 《Effective C++》中推荐的单例实现方式
// 单例，类的静态成员函数调用类的静态成员变量，用这种方式实现单例
// 如果要使用这个类，那么需要调用GetInstance函数，由于GetInstance函数是一个静态成员函数，所以调用的方式是“静态成员函数可以通过类名直接调用。”即CanBridge::GetInstance
class CanBridge
{
public:
    static CanBridge &GetInstance()
    {
        static CanBridge instance;
        return instance;
    }
    void Init() {}     // 初始化
    CanDeal canDeal_;  // CanBridge类中嵌套了CanDeal，所以单例创建CanBridge时就创建了一个包含CanDeal的对象

private:
    // 用于测试的线程，test_检查每一个数据包上发了没有，testCurrent画出电机电流和BMS获取的250ms平均电流曲线
    void test();
    void testCurrent();

    std::unique_ptr<PeriodicMemberFunction<CanBridge>> test_;
    std::unique_ptr<PeriodicMemberFunction<CanBridge>> testCurrent_;

private:
    CanBridge();
    ~CanBridge();

    std::vector<RpcRecv> rpcRxDeal_;  // rpc接收处理
};
inline CanBridge &GetCanBridge() { return CanBridge::GetInstance(); }

class HardwareSensorAPI
{
public:
    HardwareSensorAPI()
    {
        // Initialize descriptions for each power-off reason
        descriptions[PowerOffReason::BmsInitialPowerOn] = "BMS initial power-on";
        descriptions[PowerOffReason::NormalButtonShutdown] = "Normal button shutdown";
        descriptions[PowerOffReason::LowBatteryVoltageShutdown] = "Low battery voltage (low charge) software shutdown";
        descriptions[PowerOffReason::OvercurrentProtectionShutdown] = "Overcurrent (ocd) protection hardware shutdown";
        descriptions[PowerOffReason::ShortCircuitProtectionShutdown] = "Short circuit (scd) protection hardware shutdown";
        descriptions[PowerOffReason::UnderVoltageProtectionShutdown] = "Under voltage (UV) hardware shutdown";
        descriptions[PowerOffReason::OverVoltageProtectionShutdown] = "Over voltage (OV) hardware shutdown";
        descriptions[PowerOffReason::SoftwareInitiatedShutdown] = "Software initiated shutdown";
        descriptions[PowerOffReason::TotalCurrentAnomaly] = "Total current anomaly (reserved, not implemented)";
        descriptions[PowerOffReason::LoadDisconnected] = "Load disconnected";
    }

    std::string GetDescription(PowerOffReason reason)
    {
        auto it = descriptions.find(reason);
        if (it != descriptions.end()) {
            return it->second;
        }
        return "Unknown reason";
    }

    std::pair<bool, PowerInfo> SetMotorPowerOn();       // 打开电机电源
    std::pair<bool, PowerInfo> SetMotorPowerOff();      // 关闭电机电源
    std::pair<bool, PowerInfo> SetBMSPowerOffCancel();  // 取消关闭电池包对外供电
    std::pair<bool, PowerInfo>
    SetBMSPowerOff();  // 关闭BMSS对外总供电，BMS收到消息后倒计时60s关，再次期间发送BMS打开供电命令可以取消关闭，重复发送关闭BMSS对外总供电可以重置倒计时的时间
    std::pair<bool, PowerInfo> GetMotorPowerStatus();                // 读取电机电源开关情况
    std::pair<bool, PowerInfo> GetBMSPowerStatus();                  // 读取BMS对外总供电开关情况，如果为关，则倒计60s之内关闭（具体取决于发送的时间 ）
    std::pair<bool, std::vector<BoardInfo>> GetPCBNumber();          // 查询PCB的ID号
    TimestampedData<bool> GetEmergStop();                            // 获取急停信息，返回的两个数据分别是数据和获取数据的时间戳
    std::array<TimestampedData<s16>, 4> GetMidsizeBMSTemperature();  // 中型电源控制板PCB板板载温度传感器温度，单位摄氏度,0xFF代表本次获取的数据异常
    TimestampedData<s16> GetMidsizePowerPcbTemperature();            // 中型电池包的温度探头
    std::array<TimestampedData<s16>, 3> GetSmallBMSTemperature();    // 小型电池包的温度探头
    TimestampedData<s16> GetSmallBMSTemperatureMax();                // 同一时间小型电池包对个温度探头中最大的那个
    TimestampedData<s16> GetLegCurrent();                            // 左前腿电流，单位mA
    TimestampedData<s16> GetLegCurrent2();                           // 右后腿电流
    TimestampedData<s16> GetLegCurrent3();                           // 左后腿电流
    TimestampedData<s16> GetLegCurrent4();                           // 右前腿电流
    TimestampedData<s16> GetLegCurrentMax();                         // 两条前腿一段时间内最大电流
    TimestampedData<s16> GetLegCurrentMax2();                        // 两条后腿一段时间内最大电流
    TimestampedData<double> GetBC();                                 // 小型电池包对外250ms内的平均电流
    TimestampedData<double> GetBCM();                                // 本次放电过程中小型电池包对外250ms内的平均电流的最大值
    TimestampedData<double> GetSmallBMSVoltage();                    // 小型电池包对外输出总电压，单位mV
    std::vector<TimestampedData<double>> GetSmallBMSCellVoltage();   // 小型电池包电芯电压
    TimestampedData<u16> GetQuantity();                              //  获取电池电量soc
    /**
    bms上一次断电原因，只有自研电池包有断电原因上报功能，外购嘉佰达的电池包无断电原因的功能：
    00H	BMS初次开机
    01H	正常按键断电
    02H	电池电压(电量低)过低软件断电
    03H	过流（ocd）保护硬件断电
    04H	短路（scd）保护硬件断电
    05H	电池电压过低（UV）硬件断电
    06H	电池电压过高（OV）硬件断电
    07H	软件下发命令断电
    08H	总电流异常（预留，没有实现这一条）
    09H	负载被拔出 */
    PowerOffReason GetLastPowerOutageReason();             // 获取上一次断电原因，中型无此功能
    PowerOffReason GetPenultimatePowerOutageReason();      // 获取上上次断电原因
    PowerOffReason GetAntepenultimatePowerOutageReason();  // 获取上上上次断电原因
private:
    // 以下接口用于硬件收包测试
    TemperatureInfo GetTemperature();       // 功能码3 获取温度
    MotorCurrentInfo GetMotorCurrent();     // 功能码4 获取电机电流值
    DetailedBatteryInfo GetBasicBattery();  // 功能码5 获取电池包详细状态参数
    BasicBatteryInfo GetDetailedBattery();  // 功能码6 获取电池包基础状态参数

private:
    std::pair<bool, PowerInfo> SetPowerStatus(uint32_t powerIndex, bool powerState);  // 功能码1 控制各模块电源供电,一次只能发送一条报文
    std::pair<bool, PowerInfo> ReadPowerStatus(uint32_t powerIndex);                  // 功能码2 查询各模块电源供电
private:
    std::optional<PowerOffReason> ToPowerOffReason(uint8_t value)
    {
        if (value <= static_cast<uint8_t>(PowerOffReason::LoadDisconnected)) {
            return static_cast<PowerOffReason>(value);
        }
        return std::nullopt;  // 如果值不在枚举定义的范围内，返回空值
    }

    std::map<PowerOffReason, std::string> descriptions;
};

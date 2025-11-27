
#include "canBridge.hpp"

#include <array>

#include "baseline.hpp"

using namespace std;

// 构造函数
CanBridge::CanBridge()
{
    // GetLog().SetLevel("sensor", 0);  // 本模块的日志等级临时调整为0

    rpcRxDeal_.emplace_back("bs::GetBoardInfo", [this](MsgType set) { return this->canDeal_.GetBoardInfo(set); });
    rpcRxDeal_.emplace_back("bs::SetPowerState", [this](MsgType set) { return this->canDeal_.SetMotorPower(set); });
    rpcRxDeal_.emplace_back("bs::GetMotorPower", [this](MsgType set) { return this->canDeal_.GetMotorPower(set); });
    for (auto &rpc : rpcRxDeal_) {
        rpc.Connect();  // 均连接到消息内存
    }

    // 测试硬件通信包用线程
    test_ = make_unique<PeriodicMemberFunction<CanBridge>>("canBridge", 0.01, this, &CanBridge::test);
    testCurrent_ = make_unique<PeriodicMemberFunction<CanBridge>>("canBridge", 0.001, this, &CanBridge::testCurrent);

    // 测试硬件通信包用线程不打开
    // test_->Start();
    //  testCurrent_->Start();
}

// 单例的析构函数是必须的，因为程序使用exit等正常退出时候会调用析构函数
CanBridge::~CanBridge()
{
    for (auto &rpc : rpcRxDeal_) {
        rpc.DisConnect();
    }
}
void CanBridge::testCurrent()
{
    // 该函数用于PublishInterestDataA画曲线，PublishInterestDataA可能有更新。现已基本弃用testCurrent函数
    // extern void PublishInterestDataA(const Vec3<double> &a, const Vec3<double> &ra);
    // HardwareSensorAPI API;
    // static DetailedBatteryInfo BMSBattery{{0, 0}};
    // BMSBattery = API.GetBasicBattery();
    // {
    //     LOG_ERROR("功能码5 获取到新的BMS电流");
    //     LOG_ERROR("BMS短时间内的平均电流为{}mA", BMSBattery.current);
    //     Vec3<double> currentVec(BMSBattery.current, 0.0, 0.0);
    //     PublishInterestDataA(currentVec, currentVec);
    //     LOG_ERROR("本次放电过程中BMS电流最大值为{}mA", BMSBattery.currentMax);
    //     LOG_ERROR("本次放电过程中BMS温度最大值为{}", BMSBattery.tempCMax);
    //     LOG_ERROR("BMS总压为{}", BMSBattery.voltage);
    //     LOG_ERROR("BMS电芯电压为:");
    //     for (size_t i = 0; i < 12; i++) {
    //         LOG_ERROR("{} ", BMSBattery.cellVoltage[i]);
    //     }
    // }

    // extern void PublishInterestDataA(const Vec3<double> &a, const Vec3<double> &ra);

    // // 功能码4 获取电机电流值
    // LOG_ERROR("************************************************");
    // MotorCurrentInfo motorCurrent{{0, 0}};
    // motorCurrent = API.GetMotorCurrent();
    // {
    //     LOG_ERROR("功能码4 获取电机电流值");
    //     LOG_ERROR("左前腿电流{},右后腿电流为{},左后腿电流为{},左后腿电流为{} 单位mA", motorCurrent.current1, motorCurrent.current2, motorCurrent.current3,
    //     motorCurrent.current4); LOG_ERROR("两条前腿上电运行过程中的最大电流{},两条后腿上电运行过程中的最大电流{} 单位mA", motorCurrent.current1Max, motorCurrent.current3Max);
    //     Vec3<double> currentVec(motorCurrent.current1.data, motorCurrent.current2.data, BMSBattery.current.data);
    //     Vec3<double> currentVec1(motorCurrent.current1Max.data, motorCurrent.current3Max.data, BMSBattery.current.data);
    //     PublishInterestDataA(currentVec, currentVec1);
    // }
}

void CanBridge::test()
{
    // HardwareSensorAPI API;
    // std::pair<bool, std::vector<BoardInfo>> rx0;
    // std::pair<bool, PowerInfo> rx1;
    // extern void PublishInterestDataA(const Vec3<double> &a, const Vec3<double> &ra);
    // LOG_ERROR("//////////////////////////////////////////////////////////////////////////////////////////////////////");

    // // Function Code 5: Get detailed battery pack status parameters
    // LOG_ERROR("1************************************************");
    // DetailedBatteryInfo BMSBattery{{0, 0}};
    // BMSBattery = API.GetBasicBattery();
    // {
    //     LOG_ERROR("Function Code 5: Retrieved new BMS current");
    //     LOG_ERROR("BMS short-term average current: {}mA", BMSBattery.current.data);
    //     LOG_ERROR("Maximum BMS current during this discharge process: {}mA", BMSBattery.currentMax.data);
    //     LOG_ERROR("Maximum BMS temperature during this discharge process: {}", BMSBattery.tempCMax.data);
    //     LOG_ERROR("Total BMS voltage: {}", BMSBattery.voltage.data);
    //     LOG_ERROR("Number of BMS cells: {}", BMSBattery.cellNum.data);
    //     LOG_ERROR("BMS cell voltages:");
    //     for (size_t i = 0; i < 13; i++) {
    //         // std::cout << static_cast<int>(BMSBattery.cellVoltage[i].data) << std::endl;  // Convert to int for printing
    //         LOG_ERROR("{} ", BMSBattery.cellVoltage[i].data);
    //     }
    // }

    // // Function Code 3: Get temperature
    // LOG_ERROR("2************************************************");
    // TemperatureInfo temperature{{0, 0}};
    // temperature = API.GetTemperature();
    // {
    //     LOG_ERROR("Function Code 3: Retrieve temperature");
    //     LOG_ERROR("BMS probe 1 temperature: {}, BMS probe 2 temperature: {}, BMS probe 3 temperature: {}",
    //               temperature.bmsTemperature1.data,
    //               temperature.bmsTemperature2.data,
    //               temperature.bmsTemperature3.data);
    //     LOG_ERROR("Power board temperature: {}", temperature.powerPcbTemperature.data);
    //     LOG_ERROR("Power board external battery probe temperatures: Probe 1: {}, Probe 2: {}, Probe 3: {}",
    //               temperature.batteryTemperature1.data,
    //               temperature.batteryTemperature2.data,
    //               temperature.batteryTemperature3.data,
    //               temperature.batteryTemperature4.data);
    // }

    // // Function Code 4: Get motor current values
    // LOG_ERROR("************************************************");
    // MotorCurrentInfo motorCurrent{{0, 0}};
    // motorCurrent = API.GetMotorCurrent();
    // {
    //     LOG_ERROR("Function Code 4: Retrieve motor current values");
    //     LOG_ERROR("Left front leg current: {}, Right rear leg current: {}, Left rear leg current: {}, Right rear leg current: {} (unit: mA)",
    //               motorCurrent.current1.data,
    //               motorCurrent.current2.data,
    //               motorCurrent.current3.data,
    //               motorCurrent.current4.data);
    //     LOG_ERROR("Maximum current during power-up operation: Front legs: {}, Rear legs: {} (unit: mA)", motorCurrent.current1Max.data, motorCurrent.current3Max.data);
    // }

    // // Function Code 6: Get basic battery pack status parameters
    // LOG_ERROR("************************************************");
    // BasicBatteryInfo basicBattery{{0, 0}};
    // basicBattery = API.GetDetailedBattery();
    // {
    //     LOG_ERROR("Function Code 6: Retrieve basic battery pack status parameters");
    //     LOG_ERROR("Battery charge level: {}", basicBattery.quantity.data);
    //     LOG_ERROR("Last power-off reason: {}", basicBattery.lastPowerOffReason.data);
    //     LOG_ERROR("Second last power-off reason: {}", basicBattery.previousPowerOffReason.data);
    //     LOG_ERROR("Third last power-off reason: {}", basicBattery.priorPowerOffReason.data);
    // }

    // TimerTools::SleepForS(10);  // Sleep
}

// 中文版打印
//  void CanBridge::test()
//  {
//      HardwareSensorAPI API;
//      std::pair<bool, std::vector<BoardInfo>> rx0;
//      std::pair<bool, PowerInfo> rx1;
//      extern void PublishInterestDataA(const Vec3<double> &a, const Vec3<double> &ra);
//      LOG_ERROR("//////////////////////////////////////////////////////////////////////////////////////////////////////");

//     // 功能码5 获取电池包详细状态参数
//     LOG_ERROR("1************************************************");
//     DetailedBatteryInfo BMSBattery{{0, 0}};
//     BMSBattery = API.GetBasicBattery();
//     {
//         LOG_ERROR("功能码5 获取到新的BMS电流");
//         LOG_ERROR("BMS短时间内的平均电流为{}mA", BMSBattery.current.data);
//         LOG_ERROR("本次放电过程中BMS电流最大值为{}mA", BMSBattery.currentMax.data);
//         LOG_ERROR("本次放电过程中BMS温度最大值为{}", BMSBattery.tempCMax.data);
//         LOG_ERROR("BMS总压为{}", BMSBattery.voltage.data);
//         LOG_ERROR("BMS电芯个数为{}", BMSBattery.cellNum.data);
//         LOG_ERROR("BMS电芯电压为:");
//         for (size_t i = 0; i < 13; i++) {
//             std::cout << static_cast<int>(BMSBattery.cellVoltage[i].data) << std::endl;  // 转换为 int 进行打印

//             LOG_ERROR("{} ", BMSBattery.cellVoltage[i].data);
//         }
//     }

//     // // 功能码3 获取温度
//     LOG_ERROR("2************************************************");
//     TemperatureInfo temperature{{0, 0}};
//     temperature = API.GetTemperature();
//     {
//         LOG_ERROR("功能码3 获取温度");
//         LOG_ERROR("BMS探头1温度为{},BMS探头2温度为{},BMS探头3温度为{}", temperature.bmsTemperature1.data, temperature.bmsTemperature2.data, temperature.bmsTemperature3.data);
//         LOG_ERROR("电源板的温度{}", temperature.powerPcbTemperature.data);
//         LOG_ERROR("电源板获取的外购电池的探头1温度{},电源板获取的外购电池的探头2温度{},电源板获取的外购电池的探头3温度{}",
//                   temperature.batteryTemperature1.data,
//                   temperature.batteryTemperature2.data,
//                   temperature.batteryTemperature3.data,
//                   temperature.batteryTemperature4.data);
//     }

//     // 功能码4 获取电机电流值
//     LOG_ERROR("************************************************");
//     MotorCurrentInfo motorCurrent{{0, 0}};
//     motorCurrent = API.GetMotorCurrent();
//     {
//         LOG_ERROR("功能码4 获取电机电流值");
//         LOG_ERROR("左前腿电流{},右后腿电流为{},左后腿电流为{},左后腿电流为{} 单位mA",
//                   motorCurrent.current1.data,
//                   motorCurrent.current2.data,
//                   motorCurrent.current3.data,
//                   motorCurrent.current4.data);
//         LOG_ERROR("两条前腿上电运行过程中的最大电流{},两条后腿上电运行过程中的最大电流{} 单位mA", motorCurrent.current1Max.data, motorCurrent.current3Max.data);
//     }

//     // 功能码6 获取电池包基础状态参数
//     LOG_ERROR("************************************************");
//     BasicBatteryInfo basicBattery{{0, 0}};
//     basicBattery = API.GetDetailedBattery();
//     {
//         LOG_ERROR("功能码6 获取电池包基础状态参数");
//         LOG_ERROR("电量{}", basicBattery.quantity.data);
//         LOG_ERROR("bms上一次断电原因{}", basicBattery.lastPowerOffReason.data);
//         LOG_ERROR("bms上上一次断电原因{}", basicBattery.previousPowerOffReason.data);
//         LOG_ERROR("bms上上上一次断电原因{}", basicBattery.priorPowerOffReason.data);
//     }

//     TimerTools::SleepForS(10);  // 睡眠
// }

std::pair<bool, std::vector<BoardInfo>> HardwareSensorAPI::GetPCBNumber(void)
{
    std::pair<bool, std::vector<BoardInfo>> resp;
    if (RpcBlockSend("bs::GetBoardInfo", &resp, 2100) == true) {
        return {resp.first, resp.second};
    }
    return {resp.first, resp.second};
}

std::pair<bool, PowerInfo> HardwareSensorAPI::GetMotorPowerStatus() { return ReadPowerStatus(1); }

std::pair<bool, PowerInfo> HardwareSensorAPI::GetBMSPowerStatus() { return ReadPowerStatus(2); }

std::pair<bool, PowerInfo> HardwareSensorAPI::SetMotorPowerOn() { return SetPowerStatus(1, 1); }

std::pair<bool, PowerInfo> HardwareSensorAPI::SetMotorPowerOff() { return SetPowerStatus(1, 0); }

std::pair<bool, PowerInfo> HardwareSensorAPI::SetBMSPowerOffCancel() { return SetPowerStatus(2, 1); }

std::pair<bool, PowerInfo> HardwareSensorAPI::SetBMSPowerOff() { return SetPowerStatus(2, 0); }

TemperatureInfo HardwareSensorAPI::GetTemperature()
{
    TemperatureInfo temp;
    temp = GetCanBridge().GetInstance().canDeal_.GetTemperatureInfo();
    return temp;
}

MotorCurrentInfo HardwareSensorAPI::GetMotorCurrent()
{
    MotorCurrentInfo temp;
    temp = GetCanBridge().GetInstance().canDeal_.GetCurrentInfo();
    return temp;
}

DetailedBatteryInfo HardwareSensorAPI::GetBasicBattery()
{
    DetailedBatteryInfo temp;
    temp = GetCanBridge().GetInstance().canDeal_.GetDetailedBatteryInfo();
    return temp;
}

BasicBatteryInfo HardwareSensorAPI::GetDetailedBattery()
{
    BasicBatteryInfo temp;
    temp = GetCanBridge().GetInstance().canDeal_.GetBasicBatteryInfo();

    return temp;
}

TimestampedData<bool> HardwareSensorAPI::GetEmergStop()
{
    TimestampedData<bool> temp;
    temp = GetCanBridge().GetInstance().canDeal_.GetEmergStopInfo();

    return temp;
}

std::pair<bool, PowerInfo> HardwareSensorAPI::SetPowerStatus(uint32_t powerIndex, bool powerState)
{
    std::pair<bool, PowerInfo> resp;
    PowerInfo powerCom;
    powerCom.index = powerIndex;
    powerCom.state = powerState;
    if (RpcBlockSend("bs::SetPowerState", powerCom, &resp, 400) == true) {
        return {resp.first, resp.second};
    }
    return {resp.first, resp.second};
}

std::pair<bool, PowerInfo> HardwareSensorAPI::ReadPowerStatus(uint32_t powerIndex)
{
    // bool timeOut = false;
    std::pair<bool, PowerInfo> resp;
    PowerInfo powerCom;
    powerCom.index = powerIndex;

    if (RpcBlockSend("bs::GetMotorPower", powerCom, &resp, 400) == true) {
        return {resp.first, resp.second};
    }
    return {resp.first, resp.second};
}

std::array<TimestampedData<s16>, 4> HardwareSensorAPI::GetMidsizeBMSTemperature()
{
    std::array<TimestampedData<s16>, 4> temp;
    temp[0] = GetCanBridge().GetInstance().canDeal_.GetTemperatureInfo().batteryTemperature1;
    temp[1] = GetCanBridge().GetInstance().canDeal_.GetTemperatureInfo().batteryTemperature2;
    temp[2] = GetCanBridge().GetInstance().canDeal_.GetTemperatureInfo().batteryTemperature3;
    temp[3] = GetCanBridge().GetInstance().canDeal_.GetTemperatureInfo().batteryTemperature4;
    return temp;
}

TimestampedData<s16> HardwareSensorAPI::GetMidsizePowerPcbTemperature()
{
    TimestampedData<s16> temp;
    temp = GetCanBridge().GetInstance().canDeal_.GetTemperatureInfo().powerPcbTemperature;
    return temp;
}

std::array<TimestampedData<s16>, 3> HardwareSensorAPI::GetSmallBMSTemperature()
{
    std::array<TimestampedData<s16>, 3> temp;
    temp[0] = GetCanBridge().GetInstance().canDeal_.GetTemperatureInfo().bmsTemperature1;
    temp[1] = GetCanBridge().GetInstance().canDeal_.GetTemperatureInfo().bmsTemperature2;
    temp[2] = GetCanBridge().GetInstance().canDeal_.GetTemperatureInfo().bmsTemperature3;
    return temp;
}

TimestampedData<s16> HardwareSensorAPI::GetLegCurrent()
{
    TimestampedData<s16> temp;
    temp = GetCanBridge().GetInstance().canDeal_.GetCurrentInfo().current1;
    return temp;
}

TimestampedData<s16> HardwareSensorAPI::GetLegCurrent2()
{
    TimestampedData<s16> temp;
    temp = GetCanBridge().GetInstance().canDeal_.GetCurrentInfo().current2;
    return temp;
}

TimestampedData<s16> HardwareSensorAPI::GetLegCurrent3()
{
    TimestampedData<s16> temp;
    temp = GetCanBridge().GetInstance().canDeal_.GetCurrentInfo().current3;
    return temp;
}

TimestampedData<s16> HardwareSensorAPI::GetLegCurrent4()
{
    TimestampedData<s16> temp;
    temp = GetCanBridge().GetInstance().canDeal_.GetCurrentInfo().current4;
    return temp;
}

TimestampedData<s16> HardwareSensorAPI::GetLegCurrentMax()
{
    TimestampedData<s16> temp;
    temp = GetCanBridge().GetInstance().canDeal_.GetCurrentInfo().current1Max;
    return temp;
}

TimestampedData<s16> HardwareSensorAPI::GetLegCurrentMax2()
{
    TimestampedData<s16> temp;
    temp = GetCanBridge().GetInstance().canDeal_.GetCurrentInfo().current1Max;
    return temp;
}

TimestampedData<s16> HardwareSensorAPI::GetSmallBMSTemperatureMax()
{
    TimestampedData<s16> temp;
    temp = GetCanBridge().GetInstance().canDeal_.GetDetailedBatteryInfo().tempCMax;
    return temp;
}

TimestampedData<double> HardwareSensorAPI::GetSmallBMSVoltage()
{
    TimestampedData<double> temp;
    temp = GetCanBridge().GetInstance().canDeal_.GetDetailedBatteryInfo().voltage;
    return temp;
}

TimestampedData<double> HardwareSensorAPI::GetBC()
{
    TimestampedData<double> temp;
    temp = GetCanBridge().GetInstance().canDeal_.GetDetailedBatteryInfo().current;
    return temp;
}

TimestampedData<double> HardwareSensorAPI::GetBCM()
{
    TimestampedData<double> temp;
    temp = GetCanBridge().GetInstance().canDeal_.GetDetailedBatteryInfo().currentMax;
    return temp;
}

std::vector<TimestampedData<double>> HardwareSensorAPI::GetSmallBMSCellVoltage()
{
    std::vector<TimestampedData<double>> temp;
    for (size_t i = 0; i < GetCanBridge().GetInstance().canDeal_.GetDetailedBatteryInfo().cellNum.data; i++) {
        temp.push_back(GetCanBridge().GetInstance().canDeal_.GetDetailedBatteryInfo().cellVoltage[i]);
    }
    return temp;
}

TimestampedData<u16> HardwareSensorAPI::GetQuantity()
{
    TimestampedData<u16> temp;
    temp = GetCanBridge().GetInstance().canDeal_.GetBasicBatteryInfo().quantity;
    return temp;
}

PowerOffReason HardwareSensorAPI::GetLastPowerOutageReason()
{
    uint8_t rawValue = GetCanBridge().GetInstance().canDeal_.GetBasicBatteryInfo().lastPowerOffReason.data;
    auto reason = ToPowerOffReason(rawValue);
    if (reason) {
        return reason.value();
    }
    return PowerOffReason::Invalid;  // 如果转换失败，返回Invalid
}

PowerOffReason HardwareSensorAPI::GetPenultimatePowerOutageReason()
{
    uint8_t rawValue = GetCanBridge().GetInstance().canDeal_.GetBasicBatteryInfo().previousPowerOffReason.data;
    auto reason = ToPowerOffReason(rawValue);
    if (reason) {
        return reason.value();
    }
    return PowerOffReason::Invalid;  // 如果转换失败，返回Invalid
}
/**
 * @brief 获取上上上次断电原因
 *
 * @return
 */
PowerOffReason HardwareSensorAPI::GetAntepenultimatePowerOutageReason() { return PowerOffReason::Invalid; }
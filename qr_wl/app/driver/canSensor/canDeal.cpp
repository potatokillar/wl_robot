
#include "canDeal.hpp"

#include "baseline.hpp"
#include "canProtocol.hpp"

using namespace std;

// 各协议命令字段
static constexpr u8 CMD_BOARD_INFO = 0x00;                    // 主机查询从机地址
static constexpr u8 CMD_SET_POWER = 0x01;                     // 控制各模块电源供电
static constexpr u8 CMD_GET_POWER = 0x02;                     // 查询供电状态（功能码02H）
static constexpr u8 CMD_TEMPERATURE_INFO = 0x03;              // 定时温度上报（功能码03H）
static constexpr u8 CMD_CURRENT_INFO = 0x04;                  // 定时电机电流检测：（功能码04H）
static constexpr u8 CMD_BMS_DETAILED_PAEAMETERS_INFO = 0x05;  // 定时电池详细状态参数上报：（功能码05H）
static constexpr u8 CMD_BMS_BASIC_PAEAMETERS_INFO = 0x06;     // 定时电池基础状态上报（功能码06H）
static constexpr u8 CMD_EMERGSTOP_INFO = 0x10;                // 急停检测（功能码04H）

CanDeal::CanDeal()
{
    // 每一个CAN接收都需要注册，接收回调是异步的，注意互斥保护
    canProto_.AddRxCall(CMD_BOARD_INFO, [this](const SensorCanData& data) { this->RxBoardInfo(data); });
    canProto_.AddRxCall(CMD_SET_POWER, [this](const SensorCanData& data) { this->RxSetPowerState(data); });
    canProto_.AddRxCall(CMD_GET_POWER, [this](const SensorCanData& data) { this->RxGetPowerInfo(data); });
    canProto_.AddRxCall(CMD_TEMPERATURE_INFO, [this](const SensorCanData& data) { this->RxTemperatureInfo(data); });
    canProto_.AddRxCall(CMD_CURRENT_INFO, [this](const SensorCanData& data) { this->RxCurrentInfo(data); });
    canProto_.AddRxCall(CMD_BMS_DETAILED_PAEAMETERS_INFO, [this](const SensorCanData& data) { this->RxDetailedBatteryInfo(data); });
    canProto_.AddRxCall(CMD_BMS_BASIC_PAEAMETERS_INFO, [this](const SensorCanData& data) { this->RxBasicBatteryInfo(data); });
    canProto_.AddRxCall(CMD_EMERGSTOP_INFO, [this](const SensorCanData& data) { this->RxEmergStopInfo(data); });

    // 默认值
    motorOrBMSPowerInfo_.first = false;
    boardInfo_.first = false;
}

CanDeal::~CanDeal() {}

TimestampedData<bool> CanDeal::GetEmergStopInfo() { return emergStopInfo_; }

MsgType CanDeal::GetBoardInfo(const MsgType& set)
{
    boardInfo_.first = false;
    boardInfo_.second.clear();

    SensorCanData can(CMD_BOARD_INFO);
    can.add = can.boardId + can.version + can.cmd;
    for (int i = 0; i < 7; i++) {
        can.add += can.payload[i];
    }

    // 无需数据域
    canProto_.Send(can);

    std::unique_lock<std::mutex> lock(rxMutex_);
    if (cv_[CMD_BOARD_INFO].wait_for(lock, std::chrono::seconds(2), [this] { return boardInfo_.first == true; })) {
        boardInfo_.first = false;
        return std::pair<bool, std::vector<BoardInfo>>(true, boardInfo_.second);
    }

    return std::pair<bool, std::vector<BoardInfo>>(false, boardInfo_.second);
}

/**
 * @brief  CanBridge主动发消息设置供电状态，阻塞 功能码1
 * @param
 * @param in 下发的命令
 * @return
 */
MsgType CanDeal::SetMotorPower(const MsgType& in)
{
    auto set = in.GetType<PowerInfo>();
    SensorCanData can(CMD_SET_POWER);

    motorOrBMSPowerInfo_.first = false;
    motorOrBMSPowerInfo_.second = PowerInfo{};

    can.payload[0] = set.index;
    can.payload[1] = set.state;
    can.add = can.boardId + can.version + can.cmd;
    for (int i = 0; i < 7; i++) {
        can.add += can.payload[i];
    }
    canProto_.Send(can);

    // 等待数据返回，超时500ms
    std::unique_lock<std::mutex> lock(rxMutex_);
    if (cv_[CMD_SET_POWER].wait_for(lock, std::chrono::milliseconds(400), [this] { return motorOrBMSPowerInfo_.first == true; })) {
        return std::pair<bool, PowerInfo>(true, motorOrBMSPowerInfo_.second);
    }

    return std::pair<bool, PowerInfo>(false, motorOrBMSPowerInfo_.second);
}

MsgType CanDeal::GetMotorPower(const MsgType& in)
{
    auto set = in.GetType<PowerInfo>();
    SensorCanData can(CMD_GET_POWER);

    motorOrBMSPowerInfo_.first = false;

    can.payload[0] = set.kind;
    can.payload[1] = set.index;
    can.add = can.boardId + can.version + can.cmd;
    for (int i = 0; i < 7; i++) {
        can.add += can.payload[i];
    }
    canProto_.Send(can);

    // 等待数据返回，超时500ms
    std::unique_lock<std::mutex> lock(rxMutex_);
    if (cv_[CMD_GET_POWER].wait_for(lock, std::chrono::milliseconds(400), [this] { return motorOrBMSPowerInfo_.first == true; })) {
        motorOrBMSPowerInfo_.first = false;
        if ((motorOrBMSPowerInfo_.second.kind == can.payload[0]) && (motorOrBMSPowerInfo_.second.index == can.payload[1])) {
            return std::pair<bool, PowerInfo>(true, motorOrBMSPowerInfo_.second);
        } else {
            return std::pair<bool, PowerInfo>(true, motorOrBMSPowerInfo_.second);
        }
    }

    return std::pair<bool, PowerInfo>(false, motorOrBMSPowerInfo_.second);
}

TemperatureInfo CanDeal::GetTemperatureInfo() { return temperatureInfo_; }

MotorCurrentInfo CanDeal::GetCurrentInfo() { return currentInfo_; }

DetailedBatteryInfo CanDeal::GetDetailedBatteryInfo() { return detailedBatteryInfo_; }

BasicBatteryInfo CanDeal::GetBasicBatteryInfo() { return basicBatteryInfo_; }

void CanDeal::RxBoardInfo(const SensorCanData& data)
{
    // 遍历存放的值，看是否已经接收过
    for (const auto& var : boardInfo_.second) {
        if (var.canId == data.boardId) {
            // return;
        }
    }

    // 此时表示是新的板子
    BoardInfo info;
    info.canId = data.payload[0];
    boardInfo_.second.push_back(info);
    boardInfo_.first = true;
    cv_[CMD_BOARD_INFO].notify_all();
    LOG_TRACE_SENSOR(" can.payload[0] ={:#04x} ", data.payload[0]);
}

void CanDeal::RxSetPowerState(const SensorCanData& data)
{
    PowerInfo info;

    switch (data.payload[0]) {
        case 0x01:
            motorOrBMSPowerInfo_.second.powerIndexMotor = data.payload[1];
            motorOrBMSPowerInfo_.second.powerStateMotor = data.payload[2];

            break;
        case 0x02:
            motorOrBMSPowerInfo_.second.powerIndexBMS = data.payload[1];
            motorOrBMSPowerInfo_.second.BMSState = data.payload[2];

            break;
        default:
            break;
    }
    cv_[CMD_SET_POWER].notify_all();
    motorOrBMSPowerInfo_.first = true;
    LOG_TRACE_SENSOR(" can.payload[0] ={:#04x} ", data.payload[0]);
}

void CanDeal::RxGetPowerInfo(const SensorCanData& data)
{
    switch (data.payload[0]) {
        case 0x01:
            motorOrBMSPowerInfo_.second.powerIndexMotor = data.payload[1];
            motorOrBMSPowerInfo_.second.powerStateMotor = data.payload[2];

            break;
        case 0x02:
            motorOrBMSPowerInfo_.second.powerIndexBMS = data.payload[1];
            motorOrBMSPowerInfo_.second.BMSState = data.payload[2];

            break;
        default:
            break;
    }

    motorOrBMSPowerInfo_.first = true;
    LOG_TRACE_SENSOR(" can.payload[0] ={:#04x} ", data.payload[0]);
}

void CanDeal::RxTemperatureInfo(const SensorCanData& data)
{
    static bool rxFlag[3]{false};

    switch (data.payload[0]) {
        case 0x01:
            // rxFlag[0] = true;
            if (data.payload[1] != 0xFF) {
                temperatureInfo_.batteryTemperature1.data = data.payload[1] - 100;
                temperatureInfo_.batteryTemperature1.timestamp = TimerTools::GetNowTickMs();
            } else {
                temperatureInfo_.batteryTemperature1.data = 0xff;
                temperatureInfo_.batteryTemperature1.timestamp = TimerTools::GetNowTickMs();
            }
            if (data.payload[1] != 0xFF) {
                temperatureInfo_.batteryTemperature2.data = data.payload[2] - 100;
                temperatureInfo_.batteryTemperature2.timestamp = TimerTools::GetNowTickMs();
            } else {
                temperatureInfo_.batteryTemperature2.data = 0xff;
                temperatureInfo_.batteryTemperature2.timestamp = TimerTools::GetNowTickMs();
            }
            if (data.payload[1] != 0xFF) {
                temperatureInfo_.batteryTemperature3.data = data.payload[3] - 100;
                temperatureInfo_.batteryTemperature3.timestamp = TimerTools::GetNowTickMs();
            } else {
                temperatureInfo_.batteryTemperature3.data = 0xff;
                temperatureInfo_.batteryTemperature3.timestamp = TimerTools::GetNowTickMs();
            }
            if (data.payload[1] != 0xFF) {
                temperatureInfo_.batteryTemperature4.data = data.payload[4] - 100;
                temperatureInfo_.batteryTemperature4.timestamp = TimerTools::GetNowTickMs();
            } else {
                temperatureInfo_.batteryTemperature4.data = 0xff;
                temperatureInfo_.batteryTemperature4.timestamp = TimerTools::GetNowTickMs();
            }
            break;

        case 0x02:
            // rxFlag[1] = true;
            temperatureInfo_.powerPcbTemperature.data = data.payload[1] - 100;
            temperatureInfo_.powerPcbTemperature.timestamp = TimerTools::GetNowTickMs();
            break;
        case 0x03:

            temperatureInfo_.bmsTemperature1.data = data.payload[1] - 100;
            temperatureInfo_.bmsTemperature1.timestamp = TimerTools::GetNowTickMs();

            temperatureInfo_.bmsTemperature2.data = data.payload[2] - 100;
            temperatureInfo_.bmsTemperature2.timestamp = TimerTools::GetNowTickMs();

            temperatureInfo_.bmsTemperature3.data = data.payload[3] - 100;
            temperatureInfo_.bmsTemperature3.timestamp = TimerTools::GetNowTickMs();

            temperatureInfo_.bmsTemperatureState.data = data.payload[4];
            temperatureInfo_.bmsTemperatureState.timestamp = TimerTools::GetNowTickMs();
        default:
            break;
    }

    LOG_TRACE_SENSOR(" can.payload[0] ={:#04x} ", data.payload[0]);
}

void CanDeal::RxCurrentInfo(const SensorCanData& data)
{
    static bool rxFlag[3]{false};
    lock_guard<mutex> lock(rxMutex_);

    switch (data.payload[0]) {
        case 0x01:
            rxFlag[0] = true;
            currentInfo_.current1.data = (s16)(data.payload[1] << 8) | data.payload[2];
            currentInfo_.current1.timestamp = TimerTools::GetNowTickMs();
            currentInfo_.current2.data = (s16)(data.payload[4] << 8) | data.payload[5];
            currentInfo_.current2.timestamp = TimerTools::GetNowTickMs();
            break;

        case 0x03:
            rxFlag[1] = true;
            currentInfo_.current3.data = (s16)(data.payload[1] << 8) | data.payload[2];
            currentInfo_.current3.timestamp = TimerTools::GetNowTickMs();
            currentInfo_.current4.data = (s16)(data.payload[4] << 8) | data.payload[5];
            currentInfo_.current4.timestamp = TimerTools::GetNowTickMs();
            break;

        case 0x07:
            rxFlag[2] = true;
            currentInfo_.current1Max.data = (s16)(data.payload[1] << 8) | data.payload[2];
            currentInfo_.current1Max.timestamp = TimerTools::GetNowTickMs();
            currentInfo_.current3Max.data = (s16)(data.payload[4] << 8) | data.payload[5];
            currentInfo_.current3Max.timestamp = TimerTools::GetNowTickMs();
            break;

        default:
            break;
    }
    bool allReceived = true;
    for (bool received : rxFlag) {
        if (!received) {
            allReceived = false;
            break;
        }
    }
    if (allReceived == true) {
        std::fill_n(rxFlag, sizeof(rxFlag) / sizeof(rxFlag[0]), false);
    }
    LOG_TRACE_SENSOR(" can.payload[0] ={:#04x} ", data.payload[0]);
}

void CanDeal::RxDetailedBatteryInfo(const SensorCanData& data)
{
    static bool rxFlag[9]{false};

    switch (data.payload[0]) {
        case 1:
            // rxFlag[0] = true;
            if ((data.payload[1] == 0xFF) && (data.payload[2] == 0xFF)) {
                detailedBatteryInfo_.tempCMax.data = 0xFFFF;
                detailedBatteryInfo_.tempCMax.timestamp = TimerTools::GetNowTickMs();
            } else {
                detailedBatteryInfo_.tempCMax.data = (u16)(data.payload[1] << 8) | data.payload[2];
                detailedBatteryInfo_.tempCMax.data -= 100;
                detailedBatteryInfo_.tempCMax.timestamp = TimerTools::GetNowTickMs();
            }
            detailedBatteryInfo_.voltage.data = (u16)(data.payload[4] << 8) | data.payload[5];
            detailedBatteryInfo_.voltage.data = detailedBatteryInfo_.voltage.data / 1000.0;
            detailedBatteryInfo_.voltage.timestamp = TimerTools::GetNowTickMs();
            break;
        case 0x41:
            // rxFlag[1] = true;
            detailedBatteryInfo_.cellVoltage[0].data = (u16)(data.payload[1] << 8) | data.payload[2];
            detailedBatteryInfo_.cellVoltage[0].data = detailedBatteryInfo_.cellVoltage[0].data / 1000.0;
            detailedBatteryInfo_.cellVoltage[0].timestamp = TimerTools::GetNowTickMs();
            detailedBatteryInfo_.cellVoltage[1].data = (u16)(data.payload[4] << 8) | data.payload[5];
            detailedBatteryInfo_.cellVoltage[1].data = detailedBatteryInfo_.cellVoltage[1].data / 1000.0;
            detailedBatteryInfo_.cellVoltage[1].timestamp = TimerTools::GetNowTickMs();
            break;
        case 0x43:
            // rxFlag[2] = true;
            detailedBatteryInfo_.cellVoltage[2].data = (u16)(data.payload[1] << 8) | data.payload[2];
            detailedBatteryInfo_.cellVoltage[2].data = detailedBatteryInfo_.cellVoltage[2].data / 1000.0;
            detailedBatteryInfo_.cellVoltage[2].timestamp = TimerTools::GetNowTickMs();
            detailedBatteryInfo_.cellVoltage[3].data = (u16)(data.payload[4] << 8) | data.payload[5];
            detailedBatteryInfo_.cellVoltage[3].data = detailedBatteryInfo_.cellVoltage[3].data / 1000.0;
            detailedBatteryInfo_.cellVoltage[3].timestamp = TimerTools::GetNowTickMs();
            break;
        case 0x45:
            // rxFlag[3] = true;
            detailedBatteryInfo_.cellVoltage[4].data = (u16)(data.payload[1] << 8) | data.payload[2];
            detailedBatteryInfo_.cellVoltage[4].data = detailedBatteryInfo_.cellVoltage[4].data / 1000.0;
            detailedBatteryInfo_.cellVoltage[4].timestamp = TimerTools::GetNowTickMs();
            detailedBatteryInfo_.cellVoltage[5].data = (u16)(data.payload[4] << 8) | data.payload[5];
            detailedBatteryInfo_.cellVoltage[5].data = detailedBatteryInfo_.cellVoltage[5].data / 1000.0;
            detailedBatteryInfo_.cellVoltage[5].timestamp = TimerTools::GetNowTickMs();
            break;
        case 0x47:
            // rxFlag[4] = true;
            detailedBatteryInfo_.cellVoltage[6].data = (u16)(data.payload[1] << 8) | data.payload[2];
            detailedBatteryInfo_.cellVoltage[6].data = detailedBatteryInfo_.cellVoltage[6].data / 1000.0;
            detailedBatteryInfo_.cellVoltage[6].timestamp = TimerTools::GetNowTickMs();
            detailedBatteryInfo_.cellVoltage[7].data = (u16)(data.payload[4] << 8) | data.payload[5];
            detailedBatteryInfo_.cellVoltage[7].data = detailedBatteryInfo_.cellVoltage[7].data / 1000.0;
            detailedBatteryInfo_.cellVoltage[7].timestamp = TimerTools::GetNowTickMs();
            break;
        case 0x49:
            // rxFlag[5] = true;
            detailedBatteryInfo_.cellVoltage[8].data = (u16)(data.payload[1] << 8) | data.payload[2];
            detailedBatteryInfo_.cellVoltage[8].data = detailedBatteryInfo_.cellVoltage[8].data / 1000.0;
            detailedBatteryInfo_.cellVoltage[8].timestamp = TimerTools::GetNowTickMs();
            detailedBatteryInfo_.cellVoltage[9].data = (u16)(data.payload[4] << 8) | data.payload[5];
            detailedBatteryInfo_.cellVoltage[9].data = detailedBatteryInfo_.cellVoltage[9].data / 1000.0;
            detailedBatteryInfo_.cellVoltage[9].timestamp = TimerTools::GetNowTickMs();
            break;
        case 0x4B:
            // rxFlag[6] = true;
            detailedBatteryInfo_.cellVoltage[10].data = (u16)(data.payload[1] << 8) | data.payload[2];
            detailedBatteryInfo_.cellVoltage[10].data = detailedBatteryInfo_.cellVoltage[10].data / 1000.0;
            detailedBatteryInfo_.cellVoltage[10].timestamp = TimerTools::GetNowTickMs();
            detailedBatteryInfo_.cellVoltage[11].data = (u16)(data.payload[4] << 8) | data.payload[5];
            detailedBatteryInfo_.cellVoltage[11].data = detailedBatteryInfo_.cellVoltage[11].data / 1000.0;
            detailedBatteryInfo_.cellVoltage[11].timestamp = TimerTools::GetNowTickMs();
            break;
        case 0x4D:
            // rxFlag[6] = true;
            detailedBatteryInfo_.cellVoltage[12].data = (u16)(data.payload[1] << 8) | data.payload[2];
            detailedBatteryInfo_.cellVoltage[12].data = detailedBatteryInfo_.cellVoltage[12].data / 1000.0;
            detailedBatteryInfo_.cellVoltage[12].timestamp = TimerTools::GetNowTickMs();
            detailedBatteryInfo_.cellVoltage[13].data = (u16)(data.payload[4] << 8) | data.payload[5];
            detailedBatteryInfo_.cellVoltage[13].data = detailedBatteryInfo_.cellVoltage[13].data / 1000.0;
            detailedBatteryInfo_.cellVoltage[13].timestamp = TimerTools::GetNowTickMs();
            break;
        case 0x4F:
            // rxFlag[6] = true;
            detailedBatteryInfo_.cellVoltage[14].data = (u16)(data.payload[1] << 8) | data.payload[2];
            detailedBatteryInfo_.cellVoltage[14].data = detailedBatteryInfo_.cellVoltage[14].data / 1000.0;
            detailedBatteryInfo_.cellVoltage[14].timestamp = TimerTools::GetNowTickMs();
            detailedBatteryInfo_.cellVoltage[15].data = (u16)(data.payload[4] << 8) | data.payload[5];
            detailedBatteryInfo_.cellVoltage[15].data = detailedBatteryInfo_.cellVoltage[15].data / 1000.0;
            detailedBatteryInfo_.cellVoltage[15].timestamp = TimerTools::GetNowTickMs();
            break;
        case 0x03:
            // rxFlag[7] = true;
            detailedBatteryInfo_.current.data = (u16)(data.payload[1] << 8) | data.payload[2];
            detailedBatteryInfo_.current.data = detailedBatteryInfo_.current.data / 1000.0;
            detailedBatteryInfo_.current.timestamp = TimerTools::GetNowTickMs();
            break;
        case 0x04:
            // rxFlag[8] = true;
            detailedBatteryInfo_.currentMax.data = (u16)(data.payload[1] << 8) | data.payload[2];
            detailedBatteryInfo_.currentMax.data = detailedBatteryInfo_.currentMax.data / 1000.0;
            detailedBatteryInfo_.currentMax.timestamp = TimerTools::GetNowTickMs();
            break;
        case 0x40:
            detailedBatteryInfo_.cellNum.data = data.payload[1];
            detailedBatteryInfo_.cellNum.timestamp = TimerTools::GetNowTickMs();

            break;
        default:
            break;
    }
    bool allReceived = true;
    for (bool received : rxFlag) {
        if (!received) {
            allReceived = false;
            break;
        }
    }
    LOG_TRACE_SENSOR(" can.payload[0] ={:#04x} ", data.payload[0]);
}

void CanDeal::RxBasicBatteryInfo(const SensorCanData& data)
{
    lock_guard<mutex> lock(rxMutex_);

    basicBatteryInfo_.quantity.data = ((u16)(data.payload[1]) << 8) | ((u16)data.payload[2]);
    basicBatteryInfo_.quantity.timestamp = TimerTools::GetNowTickMs();
    basicBatteryInfo_.lastPowerOffReason.data = data.payload[4];
    basicBatteryInfo_.lastPowerOffReason.timestamp = TimerTools::GetNowTickMs();
    basicBatteryInfo_.previousPowerOffReason.data = data.payload[5];
    basicBatteryInfo_.previousPowerOffReason.timestamp = TimerTools::GetNowTickMs();
    basicBatteryInfo_.priorPowerOffReason.data = data.payload[6];
    basicBatteryInfo_.priorPowerOffReason.timestamp = TimerTools::GetNowTickMs();

    LOG_TRACE_SENSOR(" can.payload[0] ={:#04x} ", data.payload[0]);
}

void CanDeal::RxEmergStopInfo(const SensorCanData& data)
{
    lock_guard<mutex> lock(rxMutex_);
    if (data.payload[0] != 0) {
        emergStopInfo_.data = true;
        emergStopInfo_.timestamp = TimerTools::GetNowTickMs();
    } else {
        emergStopInfo_.data = false;
        emergStopInfo_.timestamp = TimerTools::GetNowTickMs();
    }

    // 急停需要回复同样的值
    SensorCanData can(CMD_EMERGSTOP_INFO);
    can.payload[0] = data.payload[0];
    can.add = can.boardId + can.version + can.cmd;
    for (int i = 0; i < 7; i++) {
        can.add += can.payload[i];
    }
    canProto_.Send(can);

    LOG_TRACE_SENSOR(" can.payload[0] ={:#04x} ", data.payload[0]);
}

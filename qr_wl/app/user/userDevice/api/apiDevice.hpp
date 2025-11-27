
#pragma once
#include "baseline.hpp"
#include "canBridge.hpp"
#include "gripper.hpp"
#include "sensorType.hpp"
constexpr int CTRL_HIGHEST_LEVEL = 0;
constexpr int CTRL_GAMEPAD_LEVEL = 1;
constexpr int CTRL_SDK_LEVEL = 2;
constexpr int CTRL_LOWEST_LEVEL = 100;

struct DeviceInfo
{
    std::string name = "";      // 设备名称，来自设备定制参数
    std::string serialNo = "";  // 设备序列号，来自设备定制参数
    std::string model = "";     // 设备型号，来自toml文件
    struct Version
    {
        std::string sdk = "";         // SDK版本，SDK必须>=这个版本才能支持
        std::string software = "";    // 软件tag版本，来自软件
        std::string hardware = "";    // 硬件版本，来自设备定制参数
        std::string mechanical = "";  // 结构版本，来自设备定制参数

    } version;
};

class ApiDevice
{
public:
    RobotState GetRobotState() const;

    DeviceInfo GetDeviceInfo();
    void SetMotorInit(bool set);
    void SetRobotName(const std::string &name);
    void SetRobotSerialNo(const std::string &serialNo);
    void SetRobotDescription(const std::string &description);

    std::string GetRobotDescription();

    bool CanIctrl(int level);
    bool AccessCtrlPermission(int level);
    bool ReleaseCtrlPermission(int level);

    void Shutdown();

    std::pair<bool, PowerInfo> SetMotorPowerOn();       // 打开电机电源
    std::pair<bool, PowerInfo> SetMotorPowerOff();      // 关闭电机电源
    std::pair<bool, PowerInfo> SetBMSPowerOffCancel();  // 取消关闭电池包对外供电
    std::pair<bool, PowerInfo>
    SetBMSPowerOff();  // 关闭BMSS对外总供电，BMS收到消息后倒计时60s关，再次期间发送BMS打开供电命令可以取消关闭，重复发送关闭BMSS对外总供电可以重置倒计时的时间
    std::pair<bool, PowerInfo> GetMotorPowerStatus();        // 读取电机电源开关情况
    std::pair<bool, PowerInfo> GetBMSPowerStatus();          // 读取BMS对外总供电开关情况，如果为关，则倒计60s之内关闭（具体取决于发送的时间 ）
    std::pair<bool, std::vector<BoardInfo>> GetPCBNumber();  // 查询PCB的ID号
    TimestampedData<bool> GetEmergStop();                    // 获取急停信息，返回的两个数据分别是数据和获取数据的时间戳

    std::array<TimestampedData<s16>, 4> GetMidsizeBMSTemperature();  // 中型电源控制板PCB板板载温度传感器温度，单位摄氏度,0xFF代表本次获取的数据异常
    TimestampedData<s16> GetMidsizePowerPcbTemperature();            // 中型电池包的温度探头
    std::array<TimestampedData<s16>, 3> GetSmallBMSTemperature();    // 小型电池包的温度探头
    TimestampedData<s16> GetSmallBMSTemperatureMax();                // 同一时间小型电池包对个温度探头中最大的那个
    double GetLegCurrent();                                          // 左前腿电流，单位A
    double GetLegCurrent2();                                         // 右后腿电流
    double GetLegCurrent3();                                         // 左后腿电流
    double GetLegCurrent4();                                         // 右前腿电流
    double GetLegCurrentMax();                                       // 两条前腿一段时间内最大电流
    double GetLegCurrentMax2();                                      // 两条后腿一段时间内最大电流
    double GetBC();                                                  // 小型电池包对外250ms内的平均电流
    double GetBCM();                                                 // 本次放电过程中小型电池包对外250ms内的平均电流的最大值
    double GetSmallBMSVoltage();                                     // 小型电池包对外输出总电压，单位V
    std::vector<double> GetSmallBMSCellVoltage();                    // 小型电池包电芯电压
    u16 GetQuantity();                                               //  获取电池电量soc
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
    int level_ = 100;
    bool errPrint_{false};
    HardwareSensorAPI hardwareAPI_;
};
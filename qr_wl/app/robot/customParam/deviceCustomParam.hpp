
#pragma once

#include "baseline.hpp"

class DeviceCustomParam
{
public:
    static DeviceCustomParam& GetInstance()
    {
        static DeviceCustomParam instance;
        return instance;
    }

    ParamHotUpdateServer sdkInput_{"deviceParam"};

    std::pair<bool, Mat34<double>> ReadZeroPos();
    bool WriteZeroPos(const Mat34<double>& pos);

    std::optional<std::vector<double>> ReadMotorInitPos(const std::string& tag);
    bool WriteMotorInitPos(const std::string& tag, const std::vector<double>& pos, const std::string& description = "");

    std::optional<std::array<std::array<double, 4>, 4>> ReadWheelZeroPos();
    bool WriteWheelZeroPos(const std::array<std::array<double, 4>, 4>& pos);

    std::optional<std::vector<double>> ReadHumanZeroPos(const std::string& tag = "");
    bool WriteHumanZeroPos(const std::vector<double>& pos, const std::string& tag = "");

    std::pair<bool, std::string> ReadRobotName();
    std::pair<bool, std::string> ReadRobotDescription();
    std::pair<bool, std::string> ReadSerialNo();
    std::pair<bool, std::string> ReadHardwareVer();
    std::pair<bool, std::string> ReadMechanicalVer();

    void WriteRobotName(const std::string& set);
    void WriteRobotDescription(const std::string& set);
    void WriteSerialNo(const std::string& set);
    void WriteHardwareVer(const std::string& set);
    void WriteMechanicalVer(const std::string& set);

    void UpdateBootCount();
    std::pair<bool, int> ReadBootCount();

    std::pair<bool, double> ReadInitFlag();
    bool WriteInitFlag(double set);

private:
    DeviceCustomParam();
    void Init();
    bool SetZeroPos(std::vector<double> set);
    std::vector<double> GetZeroPos();

    bool SetRobotName(std::vector<std::string> set);
    std::vector<std::string> GetRobotName();
    ParamInfo GetRobotNameInfo();

    bool SetRobotDescription(std::vector<std::string> set);
    std::vector<std::string> GetRobotDescription();
    ParamInfo GetRobotDescriptionInfo();

    bool SetSerialNo(std::vector<std::string> set);
    std::vector<std::string> GetSerialNo();
    ParamInfo GetSerialNoInfo();

    bool SetHardwareVer(std::vector<std::string> set);
    std::vector<std::string> GetHardwareVer();
    ParamInfo GetHardwareVerInfo();

    bool SetMechanicalVer(std::vector<std::string> set);
    std::vector<std::string> GetMechanicalVer();
    ParamInfo GetMechanicalVerInfo();

    bool SetInitFlag(std::vector<double> set);
    std::vector<double> GetInitFlag();
    ParamInfo GetInitFlagInfo();

    CppSqlite sql_;
};

inline DeviceCustomParam& GetDevCustomParam() { return DeviceCustomParam::GetInstance(); }
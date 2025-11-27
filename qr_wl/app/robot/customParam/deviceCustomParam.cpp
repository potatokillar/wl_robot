
#include "deviceCustomParam.hpp"

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include "baseline.hpp"
#include "deviceParam.hpp"

using namespace std;

DeviceCustomParam::DeviceCustomParam()
{
    Init();

    sdkInput_.AddParamStr(
        "robotName",
        [this](std::vector<string> set) { return this->SetRobotName(set); },
        [this]() { return this->GetRobotName(); },
        [this]() { return this->GetRobotNameInfo(); });

    sdkInput_.AddParamStr(
        "robotDescription",
        [this](std::vector<string> set) { return this->SetRobotDescription(set); },
        [this]() { return this->GetRobotDescription(); },
        [this]() { return this->GetRobotDescriptionInfo(); });

    sdkInput_.AddParamStr(
        "serialNo",
        [this](std::vector<string> set) { return this->SetSerialNo(set); },
        [this]() { return this->GetSerialNo(); },
        [this]() { return this->GetSerialNoInfo(); });

    sdkInput_.AddParamStr(
        "hardwareVer",
        [this](std::vector<string> set) { return this->SetHardwareVer(set); },
        [this]() { return this->GetHardwareVer(); },
        [this]() { return this->GetHardwareVerInfo(); });

    sdkInput_.AddParamStr(
        "mechanicalVer",
        [this](std::vector<string> set) { return this->SetMechanicalVer(set); },
        [this]() { return this->GetMechanicalVer(); },
        [this]() { return this->GetMechanicalVerInfo(); });

    sdkInput_.AddParamNum(
        "initFlag",
        [this](std::vector<double> set) { return this->SetInitFlag(set); },
        [this]() { return this->GetInitFlag(); },
        [this]() { return this->GetInitFlagInfo(); });
}

void DeviceCustomParam::Init()
{
    vector<string> initStr = {"no data"};
    vector<double> initNum = {1.0};
    if (ReadRobotName().first == false) {
        SetRobotName(initStr);
    }
    if (ReadRobotDescription().first == false) {
        SetRobotDescription(initStr);
    }
    if (ReadSerialNo().first == false) {
        SetSerialNo(initStr);
    }
    if (ReadHardwareVer().first == false) {
        SetHardwareVer(initStr);
    }
    if (ReadMechanicalVer().first == false) {
        SetMechanicalVer(initStr);
    }

    if (ReadInitFlag().first == false) {
        SetInitFlag(initNum);
    }
}

void DeviceCustomParam::UpdateBootCount()
{
    auto [ret, data] = sql_.Read("device", "bootCount");
    if (ret == false) {
        sql_.AsyncWrite("device", "bootCount", "int", "1");
    } else {
        try {
            int num = std::stoi(data);
            int numStr = num + 1;
            sql_.AsyncWrite("device", "bootCount", "int", std::to_string(numStr));
        } catch (std::exception&) {
        }
    }
}

std::pair<bool, int> DeviceCustomParam::ReadBootCount()
{
    int num = 0;
    auto [ret, data] = sql_.Read("device", "bootCount");
    if (ret == true) {
        try {
            num = std::stoi(data);
        } catch (std::exception&) {
        }
    }
    return {ret, num};
}

bool DeviceCustomParam::SetZeroPos(std::vector<double> set)
{
    if (set.size() != 12) {
        return false;
    }

    string value;
    for (auto v : set) {
        value.append(std::to_string(v));
        value.append(" ");
    }
    sql_.SyncWrite("device", "qrMotorZeroPos", "Mat34<double>", value);

    return true;
}

std::vector<double> DeviceCustomParam::GetZeroPos()
{
    std::vector<double> retData;
    retData.resize(12);

    auto [ret, data] = ReadZeroPos();
    if (ret == true) {
        std::memcpy(retData.data(), data.data(), data.size() * sizeof(double));
    }
    return retData;
}

std::pair<bool, Mat34<double>> DeviceCustomParam::ReadZeroPos()
{
    Mat34<double> zeroPos;

    auto [ret, data] = sql_.Read("device", "qrMotorZeroPos");
    if (ret == true) {
        std::istringstream iss(data);
        double number;
        vector<double> tmpData;
        while (iss >> number) {
            tmpData.push_back(number);
        }

        if (tmpData.size() == 12) {
            zeroPos << tmpData[0], tmpData[1], tmpData[2], tmpData[3], tmpData[4], tmpData[5], tmpData[6], tmpData[7], tmpData[8], tmpData[9], tmpData[10], tmpData[11];
        }
    }

    return {ret, zeroPos};
}

bool DeviceCustomParam::WriteZeroPos(const Mat34<double>& pos)
{
    std::vector<double> set;
    string value;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 4; j++) {
            value.append(std::to_string(pos(i, j)));
            value.append(" ");
        }
    }

    return sql_.SyncWrite("device", "qrMotorZeroPos", "Mat34<double>", value);
}

std::optional<std::vector<double>> DeviceCustomParam::ReadMotorInitPos(const std::string& tag)
{
    auto [ret, data] = sql_.Read("device", tag);
    if (ret == true) {
        std::istringstream iss(data);
        double number;
        vector<double> retData;
        while (iss >> number) {
            retData.push_back(number);
        }
        return retData;
    }
    return std::nullopt;
}
bool DeviceCustomParam::WriteMotorInitPos(const std::string& tag, const std::vector<double>& pos, const std::string& description)
{
    string value;

    for (size_t i = 0; i < pos.size(); i++) {
        value.append(std::to_string(pos[i]));
        value.append(" ");
    }

    return sql_.SyncWrite("device", tag, description, value);
}

std::optional<std::array<std::array<double, 4>, 4>> DeviceCustomParam::ReadWheelZeroPos()
{
    std::array<std::array<double, 4>, 4> zeroPos;

    auto [ret, data] = sql_.Read("device", "legWheelMotorZeroPos");
    if (ret == true) {
        std::istringstream iss(data);
        double number;
        vector<double> tmpData;
        while (iss >> number) {
            tmpData.push_back(number);
        }

        if (tmpData.size() == 16) {
            for (int leg = 0; leg < 4; leg++) {
                for (int motor = 0; motor < 4; motor++) {
                    zeroPos[leg][motor] = tmpData[leg * 4 + motor];
                }
            }
        }
        return zeroPos;
    }

    return std::nullopt;
}

bool DeviceCustomParam::WriteWheelZeroPos(const std::array<std::array<double, 4>, 4>& pos)
{
    std::vector<double> set;
    string value;
    for (int leg = 0; leg < 4; leg++) {
        for (int motor = 0; motor < 4; motor++) {
            value.append(std::to_string(pos[leg][motor]));
            value.append(" ");
        }
    }
    return sql_.SyncWrite("device", "legWheelMotorZeroPos", "array<double, 4, 4>", value);
}

std::optional<std::vector<double>> DeviceCustomParam::ReadHumanZeroPos(const std::string& tag)
{
    std::vector<double> zeroPos;

    auto [ret, data] = sql_.Read("device", "humanMotorZeroPos" + tag);
    if (ret == true) {
        std::istringstream iss(data);
        double number;
        vector<double> tmpData;
        while (iss >> number) {
            tmpData.push_back(number);
        }

        for (size_t motor = 0; motor < tmpData.size(); motor++) {
            zeroPos.push_back(tmpData[motor]);
        }
        return zeroPos;
    }

    return std::nullopt;
}
bool DeviceCustomParam::WriteHumanZeroPos(const std::vector<double>& pos, const std::string& tag)
{
    string value;

    for (size_t motor = 0; motor < pos.size(); motor++) {
        value.append(std::to_string(pos[motor]));
        value.append(" ");
    }

    return sql_.SyncWrite("device", "humanMotorZeroPos" + tag, "vector<double>", value);
}

std::pair<bool, std::string> DeviceCustomParam::ReadRobotName() { return sql_.Read("device", "robotName"); }
std::pair<bool, std::string> DeviceCustomParam::ReadRobotDescription() { return sql_.Read("device", "robotDescription"); }
std::pair<bool, std::string> DeviceCustomParam::ReadSerialNo() { return sql_.Read("device", "serialNo"); }
std::pair<bool, std::string> DeviceCustomParam::ReadHardwareVer() { return sql_.Read("device", "hardwareVer"); }
std::pair<bool, std::string> DeviceCustomParam::ReadMechanicalVer() { return sql_.Read("device", "mechanicalVer"); }

void DeviceCustomParam::WriteRobotName(const std::string& set)
{
    vector<string> sets;
    sets.push_back(set);
    SetRobotName(sets);
}

void DeviceCustomParam::WriteRobotDescription(const std::string& set)
{
    vector<string> sets;
    sets.push_back(set);
    SetRobotDescription(sets);
}

void DeviceCustomParam::WriteSerialNo(const std::string& set)
{
    vector<string> sets;
    sets.push_back(set);
    SetSerialNo(sets);
}

void DeviceCustomParam::WriteHardwareVer(const std::string& set)
{
    vector<string> sets;
    sets.push_back(set);
    SetHardwareVer(sets);
}

void DeviceCustomParam::WriteMechanicalVer(const std::string& set)
{
    vector<string> sets;
    sets.push_back(set);
    SetMechanicalVer(sets);
}

bool DeviceCustomParam::SetRobotName(std::vector<std::string> set)
{
    if (set.empty() == true) {
        return false;
    }

    string value = set.front();
    sql_.SyncWrite("device", "robotName", "string", value);
    return true;
}
std::vector<std::string> DeviceCustomParam::GetRobotName()
{
    vector<string> value;
    auto [ret, name] = ReadRobotName();
    if (ret == true) {
        value.push_back(name);
    }
    return value;
}
ParamInfo DeviceCustomParam::GetRobotNameInfo()
{
    ParamInfo info;
    info.name = "robotName";
    info.type = ParamType::text;
    return info;
}

bool DeviceCustomParam::SetRobotDescription(std::vector<std::string> set)
{
    if (set.empty() == true) {
        return false;
    }

    string value = set.front();
    sql_.SyncWrite("device", "robotDescription", "string", value);
    return true;
}
std::vector<std::string> DeviceCustomParam::GetRobotDescription()
{
    vector<string> value;
    auto [ret, name] = ReadRobotDescription();
    if (ret == true) {
        value.push_back(name);
    }
    return value;
}
ParamInfo DeviceCustomParam::GetRobotDescriptionInfo()
{
    ParamInfo info;
    info.name = "robotDescription";
    info.type = ParamType::text;
    return info;
}

bool DeviceCustomParam::SetSerialNo(std::vector<std::string> set)
{
    if (set.empty() == true) {
        return false;
    }

    string value = set.front();
    sql_.SyncWrite("device", "serialNo", "string", value);
    return true;
}
std::vector<std::string> DeviceCustomParam::GetSerialNo()
{
    vector<string> value;
    auto [ret, name] = ReadSerialNo();
    if (ret == true) {
        value.push_back(name);
    }
    return value;
}
ParamInfo DeviceCustomParam::GetSerialNoInfo()
{
    ParamInfo info;
    info.name = "serialNo";
    info.type = ParamType::text;
    return info;
}

bool DeviceCustomParam::SetHardwareVer(std::vector<std::string> set)
{
    if (set.empty() == true) {
        return false;
    }

    string value = set.front();
    sql_.SyncWrite("device", "hardwareVer", "string", value);
    return true;
}
std::vector<std::string> DeviceCustomParam::GetHardwareVer()
{
    vector<string> value;
    auto [ret, name] = ReadHardwareVer();
    if (ret == true) {
        value.push_back(name);
    }
    return value;
}
ParamInfo DeviceCustomParam::GetHardwareVerInfo()
{
    ParamInfo info;
    info.name = "hardwareVer";
    info.type = ParamType::text;
    return info;
}

bool DeviceCustomParam::SetMechanicalVer(std::vector<std::string> set)
{
    if (set.empty() == true) {
        return false;
    }

    string value = set.front();
    sql_.SyncWrite("device", "mechanicalVer", "string", value);
    return true;
}
std::vector<std::string> DeviceCustomParam::GetMechanicalVer()
{
    vector<string> value;
    auto [ret, name] = ReadMechanicalVer();
    if (ret == true) {
        value.push_back(name);
    }
    return value;
}
ParamInfo DeviceCustomParam::GetMechanicalVerInfo()
{
    ParamInfo info;
    info.name = "mechanicalVer";
    info.type = ParamType::text;
    return info;
}

std::pair<bool, double> DeviceCustomParam::ReadInitFlag()
{
    double data = 0;
    auto ret = sql_.Read("device", "initFlag");
    if (ret.first == true) {
        try {
            data = stoi(ret.second);
        } catch (std::exception&) {
        }
    }
    return {ret.first, data};
}
bool DeviceCustomParam::WriteInitFlag(double set) { return sql_.SyncWrite("device", "initFlag", "string", std::to_string(set)); }
bool DeviceCustomParam::SetInitFlag(std::vector<double> set)
{
    if (set.empty() == true) {
        return false;
    }

    return WriteInitFlag(set.front());
}
std::vector<double> DeviceCustomParam::GetInitFlag()
{
    vector<double> value;
    auto [ret, name] = ReadInitFlag();
    if (ret == true) {
        value.push_back(name);
    }
    return value;
}
ParamInfo DeviceCustomParam::GetInitFlagInfo()
{
    ParamInfo info;
    info.name = "initFlag";
    info.type = ParamType::restrict;
    return info;
}
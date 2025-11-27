
#include "debugJpos.hpp"

#include <iostream>

#include "baseline.hpp"
#include "checker.hpp"
#include "ctrlSendData.hpp"

void DebugCtrlSendData::SetPid(const std::vector<double>& set, Mat34<double>* pid)
{
    (*pid)(0, 0) = set[0];
    (*pid)(1, 0) = set[1];
    (*pid)(2, 0) = set[2];

    (*pid)(0, 1) = set[3];
    (*pid)(1, 1) = set[4];
    (*pid)(2, 1) = set[5];

    (*pid)(0, 2) = set[6];
    (*pid)(1, 2) = set[7];
    (*pid)(2, 2) = set[8];

    (*pid)(0, 3) = set[9];
    (*pid)(1, 3) = set[10];
    (*pid)(2, 3) = set[11];
}

std::vector<double> DebugCtrlSendData::GetPid(const Mat34<double>& pid)
{
    std::vector<double> set;
    set.push_back(pid(0, 0));
    set.push_back(pid(1, 0));
    set.push_back(pid(2, 0));

    set.push_back(pid(0, 1));
    set.push_back(pid(1, 1));
    set.push_back(pid(2, 1));

    set.push_back(pid(0, 2));
    set.push_back(pid(1, 2));
    set.push_back(pid(2, 2));

    set.push_back(pid(0, 3));
    set.push_back(pid(1, 3));
    set.push_back(pid(2, 3));
    return set;
}

ParamInfo DebugCtrlSendData::GetRange(const ValueRange<double>& range)
{
    ParamInfo info;
    info.type = ParamType::range;
    info.num.push_back(range.min);
    info.num.push_back(range.max);
    return info;
}

bool DebugCtrlSendData::CheckKp(const std::vector<double>& set)
{
    if (set.size() != 12) {
        return false;
    }

    for (const auto& var : set) {
        if ((var < kpRange_.min) || (var > kpRange_.max)) {
            return false;
        }
    }
    return true;
}
bool DebugCtrlSendData::CheckKd(const std::vector<double>& set)
{
    if (set.size() != 12) {
        return false;
    }

    for (const auto& var : set) {
        if ((var < kdRange_.min) || (var > kdRange_.max)) {
            return false;
        }
    }
    return true;
}
bool DebugCtrlSendData::CheckKf(const std::vector<double>& set)
{
    if (set.size() != 12) {
        return false;
    }

    for (const auto& var : set) {
        if ((var < kfRange_.min) || (var > kfRange_.max)) {
            return false;
        }
    }
    return true;
}

//-------------stand-------------///
bool DebugCtrlSendData::SetStandKp(std::vector<double> set)
{
    if (CheckKp(set) == false) {
        return false;
    }

    auto pid = &obj_->motorPid[Enum2Num(AlgoState::stand)].k2;
    SetPid(set, pid);
    return true;
}
std::vector<double> DebugCtrlSendData::GetStandKp()
{
    auto& pid = obj_->motorPid[Enum2Num(AlgoState::stand)].k2;
    return GetPid(pid);
}
ParamInfo DebugCtrlSendData::GetStandKpInfo() { return GetRange(kpRange_); }

bool DebugCtrlSendData::SetStandKd(std::vector<double> set)
{
    if (CheckKd(set) == false) {
        return false;
    }

    auto pid = &obj_->motorPid[Enum2Num(AlgoState::stand)].k1;
    SetPid(set, pid);
    return true;
}
std::vector<double> DebugCtrlSendData::GetStandKd()
{
    auto& pid = obj_->motorPid[Enum2Num(AlgoState::stand)].k1;
    return GetPid(pid);
}
ParamInfo DebugCtrlSendData::GetStandKdInfo() { return GetRange(kdRange_); }

bool DebugCtrlSendData::SetStandKf(std::vector<double> set)
{
    if (CheckKf(set) == false) {
        return false;
    }

    auto pid = &obj_->motorPid[Enum2Num(AlgoState::stand)].k3;
    SetPid(set, pid);

    return true;
}
std::vector<double> DebugCtrlSendData::GetStandKf()
{
    auto& pid = obj_->motorPid[Enum2Num(AlgoState::stand)].k3;
    return GetPid(pid);
}
ParamInfo DebugCtrlSendData::GetStandKfInfo() { return GetRange(kfRange_); }

//------------walk-----------------//
bool DebugCtrlSendData::SetWalkKp(std::vector<double> set)
{
    if (CheckKp(set) == false) {
        return false;
    }

    auto pid = &obj_->motorPid[Enum2Num(AlgoState::walk)].k2;
    SetPid(set, pid);
    return true;
}
std::vector<double> DebugCtrlSendData::GetWalkKp()
{
    auto& pid = obj_->motorPid[Enum2Num(AlgoState::walk)].k2;
    return GetPid(pid);
}
ParamInfo DebugCtrlSendData::GetWalkKpInfo() { return GetRange(kpRange_); }

bool DebugCtrlSendData::SetWalkKd(std::vector<double> set)
{
    if (CheckKd(set) == false) {
        return false;
    }

    auto pid = &obj_->motorPid[Enum2Num(AlgoState::walk)].k1;
    SetPid(set, pid);
    return true;
}
std::vector<double> DebugCtrlSendData::GetWalkKd()
{
    auto& pid = obj_->motorPid[Enum2Num(AlgoState::walk)].k1;
    return GetPid(pid);
}
ParamInfo DebugCtrlSendData::GetWalkKdInfo() { return GetRange(kdRange_); }

bool DebugCtrlSendData::SetWalkKf(std::vector<double> set)
{
    if (CheckKf(set) == false) {
        return false;
    }

    auto pid = &obj_->motorPid[Enum2Num(AlgoState::walk)].k3;
    SetPid(set, pid);

    return true;
}
std::vector<double> DebugCtrlSendData::GetWalkKf()
{
    auto& pid = obj_->motorPid[Enum2Num(AlgoState::walk)].k3;
    return GetPid(pid);
}
ParamInfo DebugCtrlSendData::GetWalkKfInfo() { return GetRange(kfRange_); }

//---------------standup-----------------------//
bool DebugCtrlSendData::SetStandUpKp(std::vector<double> set)
{
    if (CheckKp(set) == false) {
        return false;
    }

    auto pid = &obj_->motorPid[Enum2Num(AlgoState::stand2)].k2;
    SetPid(set, pid);
    return true;
}
std::vector<double> DebugCtrlSendData::GetStandUpKp()
{
    auto& pid = obj_->motorPid[Enum2Num(AlgoState::stand2)].k2;
    return GetPid(pid);
}
ParamInfo DebugCtrlSendData::GetStandUpKpInfo() { return GetRange(kpRange_); }

bool DebugCtrlSendData::SetStandUpKd(std::vector<double> set)
{
    if (CheckKd(set) == false) {
        return false;
    }

    auto pid = &obj_->motorPid[Enum2Num(AlgoState::stand2)].k1;
    SetPid(set, pid);
    return true;
}
std::vector<double> DebugCtrlSendData::GetStandUpKd()
{
    auto& pid = obj_->motorPid[Enum2Num(AlgoState::stand2)].k1;
    return GetPid(pid);
}
ParamInfo DebugCtrlSendData::GetStandUpKdInfo() { return GetRange(kdRange_); }

bool DebugCtrlSendData::SetStandUpKf(std::vector<double> set)
{
    if (CheckKf(set) == false) {
        return false;
    }

    auto pid = &obj_->motorPid[Enum2Num(AlgoState::stand2)].k3;
    SetPid(set, pid);

    return true;
}
std::vector<double> DebugCtrlSendData::GetStandUpKf()
{
    auto& pid = obj_->motorPid[Enum2Num(AlgoState::stand2)].k3;
    return GetPid(pid);
}
ParamInfo DebugCtrlSendData::GetStandUpKfInfo() { return GetRange(kfRange_); }

/**
 * @description: qInit参数可用sdk修改，程序重启后生效
 * @param set 若不是12个，则不会写入数据库，但通常先读取，必然是12个数据
 * @return {}
 */
bool DebugCtrlSendData::SetqInit(std::vector<double> set)
{
    if (set.size() != 12) {
        return false;
    }

    CppSqlite sql;
    std::string value;
    for (auto v : set) {
        value.append(std::to_string(v));
        value.append(" ");
    }

    return sql.SyncWrite("quadruped", "qInit", "Mat34<double>", value);
}
/**
 * @description: 获取qInit信息，来自qrParam(数据库的qInit会在初始化时写入qrParam)
 * @return {}
 */
std::vector<double> DebugCtrlSendData::GetqInit()
{
    auto qInit = GetQrParam().qInit;

    return Mat34ToVec(qInit);
}
ParamInfo DebugCtrlSendData::GetqInitInfo()
{
    ParamInfo info;
    info.type = ParamType::range;
    info.num.push_back(-100);
    info.num.push_back(100);
    return info;
}


#pragma once
#include <vector>

#include "baseline.hpp"
#include "debug.hpp"

class CtrlSendData;
class DebugCtrlSendData
{
public:
    void Init(CtrlSendData* set)
    {
        obj_ = set;

        // 区分各个电机的pid
        contrParamHotUpdate.AddParamNum(
            "kp-stand",
            [this](std::vector<double> set) { return this->SetStandKp(set); },
            [this]() { return this->GetStandKp(); },
            [this]() { return this->GetStandKpInfo(); });
        contrParamHotUpdate.AddParamNum(
            "kd-stand",
            [this](std::vector<double> set) { return this->SetStandKd(set); },
            [this]() { return this->GetStandKd(); },
            [this]() { return this->GetStandKdInfo(); });
        contrParamHotUpdate.AddParamNum(
            "kf-stand",
            [this](std::vector<double> set) { return this->SetStandKf(set); },
            [this]() { return this->GetStandKf(); },
            [this]() { return this->GetStandKfInfo(); });

        contrParamHotUpdate.AddParamNum(
            "kp-walk",
            [this](std::vector<double> set) { return this->SetWalkKp(set); },
            [this]() { return this->GetWalkKp(); },
            [this]() { return this->GetWalkKpInfo(); });
        contrParamHotUpdate.AddParamNum(
            "kd-walk",
            [this](std::vector<double> set) { return this->SetWalkKd(set); },
            [this]() { return this->GetWalkKd(); },
            [this]() { return this->GetWalkKdInfo(); });
        contrParamHotUpdate.AddParamNum(
            "kf-walk",
            [this](std::vector<double> set) { return this->SetWalkKf(set); },
            [this]() { return this->GetWalkKf(); },
            [this]() { return this->GetWalkKfInfo(); });

        contrParamHotUpdate.AddParamNum(
            "kp-standUp",
            [this](std::vector<double> set) { return this->SetStandUpKp(set); },
            [this]() { return this->GetStandUpKp(); },
            [this]() { return this->GetStandUpKpInfo(); });
        contrParamHotUpdate.AddParamNum(
            "kd-standUp",
            [this](std::vector<double> set) { return this->SetStandUpKd(set); },
            [this]() { return this->GetStandUpKd(); },
            [this]() { return this->GetStandUpKdInfo(); });
        contrParamHotUpdate.AddParamNum(
            "kf-standUp",
            [this](std::vector<double> set) { return this->SetStandUpKf(set); },
            [this]() { return this->GetStandUpKf(); },
            [this]() { return this->GetStandUpKfInfo(); });
        contrParamHotUpdate.AddParamNum(
            "qInit",
            [this](std::vector<double> set) { return this->SetqInit(set); },
            [this]() { return this->GetqInit(); },
            [this]() { return this->GetqInitInfo(); });
    }

    bool SetStandKp(std::vector<double> set);
    std::vector<double> GetStandKp();
    ParamInfo GetStandKpInfo();

    bool SetStandKd(std::vector<double> set);
    std::vector<double> GetStandKd();
    ParamInfo GetStandKdInfo();

    bool SetStandKf(std::vector<double> set);
    std::vector<double> GetStandKf();
    ParamInfo GetStandKfInfo();
    /////////////////////////
    bool SetWalkKp(std::vector<double> set);
    std::vector<double> GetWalkKp();
    ParamInfo GetWalkKpInfo();

    bool SetWalkKd(std::vector<double> set);
    std::vector<double> GetWalkKd();
    ParamInfo GetWalkKdInfo();

    bool SetWalkKf(std::vector<double> set);
    std::vector<double> GetWalkKf();
    ParamInfo GetWalkKfInfo();
    //////////////////////////////
    bool SetStandUpKp(std::vector<double> set);
    std::vector<double> GetStandUpKp();
    ParamInfo GetStandUpKpInfo();

    bool SetStandUpKd(std::vector<double> set);
    std::vector<double> GetStandUpKd();
    ParamInfo GetStandUpKdInfo();

    bool SetStandUpKf(std::vector<double> set);
    std::vector<double> GetStandUpKf();
    ParamInfo GetStandUpKfInfo();

    bool SetqInit(std::vector<double> set);
    std::vector<double> GetqInit();
    ParamInfo GetqInitInfo();

private:
    CtrlSendData* obj_;
    ValueRange<double> kpRange_{0, 50000};
    ValueRange<double> kdRange_{0, 500};
    ValueRange<double> kfRange_{0, 5};

    void SetPid(const std::vector<double>& set, Mat34<double>* pid);
    std::vector<double> GetPid(const Mat34<double>& pid);
    ParamInfo GetRange(const ValueRange<double>& range);
    bool CheckKp(const std::vector<double>& set);
    bool CheckKd(const std::vector<double>& set);
    bool CheckKf(const std::vector<double>& set);
};
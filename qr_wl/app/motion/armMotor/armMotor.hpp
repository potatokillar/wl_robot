
#pragma once
#include "baseline.hpp"

using TxRxFuncType = std::function<std::optional<msg::arm_cmd>(msg::arm_data)>;
using InfoFuncType = std::function<void(msg::arm_motor_info)>;

enum class ArmMotorParamType
{
    circleCnt,  // 每一圈的cnt值
};

class ArmMotor
{
public:
    ArmMotor() {};
    virtual ~ArmMotor() {};
    virtual void Start() = 0;
    virtual void Stop() = 0;

    virtual void Enable() {};
    virtual void Disable() {};
    virtual bool SetParam(ArmMotorParamType type, std::any param);
    virtual void SetIoOut(const std::string& name, std::bitset<32> set);
    virtual std::bitset<32> GetIoIn(const std::string& name);
    virtual std::bitset<32> GetIoOut(const std::string& name);

    void SetTxRxCallback(TxRxFuncType func);
    void SetInfoCallback(InfoFuncType func);

protected:
    virtual void Run() = 0;
    void UpdateBasicData();

    TxRxFuncType txRxFunc_;
    InfoFuncType infoFunc_;
    msg::arm_motor_info motorInfos_;
    bool infoIsUpdate_{false};  // info是否更新

    std::mutex mtx_;
    msg::arm_cmd motorCmd_;
    msg::arm_data motorRet_;
    bool motorCmd_inited_;
};
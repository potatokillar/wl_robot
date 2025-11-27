
#include "armMotor.hpp"

using namespace std;

/**
 * @description: 设置数据发送和接收回调
 * @param func
 * @return {}
 */

void ArmMotor::SetTxRxCallback(TxRxFuncType func) { txRxFunc_ = func; }

/**
 * @description: 设置电机信息回调，改变时会上报
 * @param func
 * @return {}
 */
void ArmMotor::SetInfoCallback(InfoFuncType func) { infoFunc_ = func; }

/**
 * @description: 设置参数
 * @param type
 * @param param
 * @return {}
 */
bool ArmMotor::SetParam(ArmMotorParamType type, std::any param)
{
    (void)type;
    (void)param;
    return true;
}

/**
 * @description: 这里是保存ethercat的IO数据
 * @return {}
 */
void ArmMotor::SetIoOut(const std::string& name, std::bitset<32> set)
{
    (void)name;
    (void)set;
}

std::bitset<32> ArmMotor::GetIoIn(const std::string& name)
{
    (void)name;
    return 0;
}

std::bitset<32> ArmMotor::GetIoOut(const std::string& name)
{
    (void)name;
    return 0;
}

/**
 * @description: 更新必要的基本数据
 * @return {}
 */
void ArmMotor::UpdateBasicData()
{
    lock_guard lock(mtx_);
    // 调用回调函数，进行发送
    if (txRxFunc_) {
        auto cmd = txRxFunc_(motorRet_);
        if (cmd) {
            motorCmd_ = cmd.value();
            // 电机正确初始化标志位
            motorCmd_inited_ = true;
        }
    }

    // 上报info信息，改变上报，低频上报
    if ((infoFunc_) && (infoIsUpdate_)) {
        infoFunc_(motorInfos_);
        infoIsUpdate_ = false;
        cout << "enable is or not " << Enum2Num(motorInfos_.state) << endl;
    }
}

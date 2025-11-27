
#include "fsmLie.hpp"

#include "baseline.hpp"
using namespace std;

void FsmLie::OnEnter()
{
    LOG_INFO("lie enter");
    jpData_->pFootForStand = qrParam.pFoot;
    done_ = true;
    // std::cout << "qInitBias =  " << qInitBias << std::endl;
}

void FsmLie::Run()
{
    // todo 动作检查
    if (qrParam.model.contains(QrModel::linkV2_3_w) || qrParam.model.contains(QrModel::gazebo_w)) {
        {
            // CmdVal4 motorCmd;
            // motorCmd.alpha = rxData_->GetMotorDataWheel().alpha;
            // motorCmd.torq.setZero();
            // motorCmd.blta.setZero();
            // txData_->SendMotorCmd(motorCmd);
        }

        {
            DataValue2 motorCmdPid;
            motorCmdPid.alpha = rxData_->GetMotorDataWheel().alpha;
            motorCmdPid.torq.setZero();
            motorCmdPid.blta.setZero();
            motorCmdPid.k2.setZero();
            motorCmdPid.k1.setZero();
            txData_->SendMotorCmd(motorCmdPid);
        }
    } else {
        DataCmd motorCmd;
        motorCmd.alpha = rxData_->GetData3().alpha;
        motorCmd.torq.setZero();
        motorCmd.blta.setZero();
        txData_->SendMotorCmd(motorCmd);
    }
}

void FsmLie::OnExit() { LOG_INFO("lie exit"); }
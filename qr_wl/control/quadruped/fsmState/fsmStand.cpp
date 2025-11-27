
#include "fsmStand.hpp"

#include "baseline.hpp"
#include "eventMsg.hpp"
#include "orientation_tools.h"

using namespace std;
using namespace ori;

// 该类继承于FsmState，构造函数写成这样
FsmStand::FsmStand(ContrContainer contr, std::shared_ptr<JpData> jpData) : FsmState(AlgoState::stand, contr, jpData)
{
    pFootOld = qrParam.pFoot;
    qdCmdFilter.setZero();
    if (qrParam.model.contains(QrModel::middleV3)) {
        kp_ = 0.005;  // 中型的这个参数小一点
    }
    // 初始化
    jpData_->qCmdRecord = mathFun_.InverseKinematics_new(jpData_->pFoot, qrParam.pRoll);  // 初始化
    Mat34<double> tmpForceFoot;
    tmpForceFoot.setZero();
    double k = 0.6;  // 偏向于后腿
    tmpForceFoot(2, 0) = -qrParam.DOG_M * G(2) * 0.5 * (1 - k);
    tmpForceFoot(2, 2) = -qrParam.DOG_M * G(2) * 0.5 * k;

    tmpForceFoot(2, 1) = tmpForceFoot(2, 0);
    tmpForceFoot(2, 3) = tmpForceFoot(2, 2);

    jpData_->tauCmdRecord = mathFun_.JacobianT(tmpForceFoot, jpData_->qCmdRecord);  // 初始化记录
    fzFilter.fill(-30);
    timeForMode = TimerTools::GetNowTickMs();
}

void FsmStand::OnEnter()
{
    LOG_INFO("stand enter");
    staEstimator_->Init();
    jpData_->contactFlag.fill(true);
    jpData_->desire.rpy.setZero();
    qdCmdFilter.setZero();
    smoothValue = 0.0;
    cmd_->ClearMotorCmdQueue();
    done_ = true;
    fzFilter.fill(-30);
    jpData_->qCmdRecord = rxData_->GetData3().alpha;
    jpData_->tauCmdRecord = rxData_->GetData3().blta;
}

void FsmStand::Run()
{
    DataCmd motorCmd;
    jpData_->desire.h += kp_ * (cmd_->GetHeight() - jpData_->desire.h);
    double kh = 1.0;
    if (jpData_->desire.h > qrParam.bodyHeight) {
        kh = mathFun_.SetValueRange(1.0 - (jpData_->desire.h - qrParam.bodyHeight) * 14.92, 0.0, 1.0, "kh limit", false);
    }
    jpData_->desire.rpy += kp_ * (cmd_->GetPose() * kh - jpData_->desire.rpy);
    // cout << "kp= " << kp << endl;
    // cout << "pose  = " << cmd_->GetPose() << endl;

    Mat34<double> tempVecFoot = jpData_->pFootForStand;

    for (int i = 0; i < 4; i++) {
        tempVecFoot(2, i) = -jpData_->desire.h;
        tempVecFoot.col(i) = mathFun_.Rx(tempVecFoot.col(i), -jpData_->desire.rpy(0));
        tempVecFoot.col(i) = mathFun_.Ry(tempVecFoot.col(i), -jpData_->desire.rpy(1));
        tempVecFoot.col(i) = mathFun_.Rz(tempVecFoot.col(i), -jpData_->desire.rpy(2));
    }
    // motorCmd.alpha = mathFun_.InverseKinematics(tempVecFoot, qrParam.pRoll);  // stand
    motorCmd.alpha = mathFun_.InverseKinematics_new(tempVecFoot, qrParam.pRoll);  //
    // 计算tau
    Mat34<double> tmpForceFoot;
    tmpForceFoot.setZero();

    double k = 0.6;
    tmpForceFoot(2, 0) = -qrParam.DOG_M * G(2) * 0.5 * (1 - k);
    tmpForceFoot(2, 2) = -qrParam.DOG_M * G(2) * 0.5 * k;

    tmpForceFoot(2, 1) = tmpForceFoot(2, 0);
    tmpForceFoot(2, 3) = tmpForceFoot(2, 2);

    motorCmd.blta = mathFun_.JacobianT(tmpForceFoot, motorCmd.alpha);
    // 计算速度命令
    Mat34<double> vFoot = (tempVecFoot - pFootOld) / qrParam.dt;
    Mat34<double> qdTmp = mathFun_.JacobianInv(vFoot, motorCmd.alpha);
    pFootOld = tempVecFoot;

    double kFilter = 0.9;
    qdCmdFilter = kFilter * qdCmdFilter + (1 - kFilter) * qdTmp;
    motorCmd.torq = qdCmdFilter;

    if (smoothValue < 1.0) {
        smoothValue += 0.003;
        motorCmd.alpha = jpData_->qCmdRecord + smoothValue * (motorCmd.alpha - jpData_->qCmdRecord);
        motorCmd.torq = smoothValue * motorCmd.torq;
        motorCmd.blta = jpData_->tauCmdRecord + smoothValue * (motorCmd.blta - jpData_->tauCmdRecord);
        // cout << jpData_->qCmdRecord << endl;
        //  cout << motorCmd.blta << endl;
        //  cout << jpData_->tauCmdRecord << endl;
    } else {
        // smoothValue = 1.0;
        jpData_->qCmdRecord = motorCmd.alpha;
        jpData_->tauCmdRecord = motorCmd.blta;
    }
    jpData_->pFootPeriodEnd = tempVecFoot;
    jpData_->pFoot = tempVecFoot;

    // 跳舞代码
    auto danceQ = cmd_->GetMotorCmdQueue();
    if (danceQ->empty() == false) {
        if (cmdQueueIdx_ < danceQ->size()) {
            // 定时从序列中读取，写入
            auto oneCmd = danceQ->at(cmdQueueIdx_);
            cmdQueueIdx_++;

            txData_->SendMotorCmd(oneCmd);
        } else {
            MsgTrySend("qr::SetMotorCmdQComplete", RetState::ok);
            cmdQueueIdx_ = 0;
            cmd_->ClearMotorCmdQueue();
            timeForMode = TimerTools::GetNowTickMs();
        }

    } else {
        double k = 0.005;
        Vec4<double> tmpFz = staEstimator_->GetResult().fFoot.row(2);
        fzFilter = (1 - k) * fzFilter + k * tmpFz;
        int missHoldN = 0;
        for (int i = 0; i < 4; i++) {
            if (fzFilter(i) > -15) {
                missHoldN++;
            }
        }
        // 力用来处理慢慢放倒的情况，加速度用来处理猛踢一脚的情况
        // if (TimerTools::GetNowTickMs() - timeForMode > 1000) {
        //     if (missHoldN >= 2 || fabs(staEstimator_->GetResult().aCom(0)) > 0.4 || fabs(staEstimator_->GetResult().aCom(1)) > 0.2) {
        //         SetQrEvent(QrEventType::unstable);
        //     }
        // }

        if (qrParam.model.contains(QrModel::linkV2_3_w) || qrParam.model.contains(QrModel::gazebo_w)) {
            DataValue2 newCmd;
            newCmd.alpha.block(0, 0, 3, 4) = motorCmd.alpha;
            newCmd.torq.block(0, 0, 3, 4) = motorCmd.torq;
            newCmd.blta.block(0, 0, 3, 4) = motorCmd.blta;

            newCmd.torq.row(3).setZero();
            newCmd.blta.row(3).setZero();

            newCmd.k2.fill(50);
            newCmd.k1.fill(3);
            newCmd.k3.fill(1.0);
            newCmd.k2.row(3).fill(0);
            newCmd.k1.row(3).fill(2);
            newCmd.k3.row(3).fill(0);

            txData_->SendMotorCmd(newCmd);
        } else {
            txData_->SendMotorCmd(motorCmd);
        }

        // 感兴趣数据
        /*
        Vec3<double> a, b, c, ra, rb, rc;
        a.setZero();
        b.setZero();
        c.setZero();
        ra.setZero();
        rb.setZero();
        rc.setZero();
        a(0) = fzFilter(0);
        ra(0) = staEstimator_->GetResult().fFoot(2, 0);

        PublishInterestData(a, ra, b, rb, c, rc);
        */
    }
}

void FsmStand::OnExit() { LOG_INFO("stand exit"); }
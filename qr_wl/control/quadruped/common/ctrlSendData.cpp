
#include "ctrlSendData.hpp"

#include "baseline.hpp"
#include "eventMsg.hpp"
#include "mathTools.hpp"

using namespace std;
/**
 * @description: 构造函数
 * @param {*}
 * @return {*}
 */
CtrlSendData::CtrlSendData(bool fromSim) : fromSim_(fromSim)
{
    // fromSim_ = false;  // 永远是真实
    motorCmdMitOld_.alpha = transform.my2mit(qrParam.qInit);
    motorCmdOld_.alpha = qrParam.qInit;

    for (auto i = 0; i < STATE_TOTALNUM; i++) {
        motorPid[i] = qrParam.motorPara[i];
    }
    debug_.Init(this);

    MathFun mathFun;
    motorCmdStandard_.alpha = mathFun.InverseKinematics(qrParam.pFoot, qrParam.pRoll);

    qLimitMit_.max = transform.my2mit(qrParam.qLimit.max);
    qLimitMit_.min = transform.my2mit(qrParam.qLimit.min);

    qdLimitMit_.max = qrParam.qdLimit.max;
    qdLimitMit_.min = qrParam.qdLimit.min;

    tauLimitMit_.max = qrParam.tauLimit.max;
    tauLimitMit_.min = qrParam.tauLimit.min;
    qLimitUnitreeMarkerMin_ << -0.9, -0.9, -0.9, -0.9, -0.35, -0.35, -0.35, -0.35, -2.55, -2.55, -2.55, -2.55;
    qLimitUnitreeMarkerMax_ << 0.9, 0.9, 0.9, 0.9, 3.5, 3.5, 3.5, 3.5, -0.92, -0.92, -0.92, -0.92;

    motorCmdUnitreeOld_.alpha = motorCmdStandard_.alpha;
    motorCmdUnitreeOld_.alpha.row(2) = -motorCmdUnitreeOld_.alpha.row(2);  // 算法坐标系和宇树坐标系三关节差负号
    for (int i = 0; i < 4; i++) {
        motorCmdUnitreeOld_.alpha(1, i) -= 1.57;  // 算法坐标系和宇树坐标系差90度差负号
        motorCmdUnitreeOld_.alpha(1, i) *= -1;
    }

    motorCmdUnitreeOld_.torq.setZero();
}

/**
 * @description: 检查命令，若有问题，则替换为上一次命令
 * @param *motorCmd
 * @return {}
 */
void CtrlSendData::CheckCmd(CmdVal2 *motorCmd)
{
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 4; j++) {
            // 对Nan的处理非常关键，防止程序崩溃
            if (std::isnan(motorCmd->alpha(i, j))) {
                motorCmd->alpha(i, j) = motorCmdOld_.alpha(i, j);
            }
            if (std::isnan(motorCmd->torq(i, j))) {
                motorCmd->torq.setZero();  // 对Nan的处理非常关键，防止程序崩溃
            }
            if (std::isnan(motorCmd->blta(i, j))) {
                motorCmd->blta.setZero();  // 对Nan的处理非常关键，防止程序崩溃
            }

            if (motorCmd->alpha(i, j) > qrParam.qLimit.max(i, j)) {
                motorCmd->alpha(i, j) = qrParam.qLimit.max(i, j);
            } else if (motorCmd->alpha(i, j) < qrParam.qLimit.min(i, j)) {
                motorCmd->alpha(i, j) = qrParam.qLimit.min(i, j);
            }
            if (motorCmd->blta(i, j) > qrParam.tauLimit.max(i, j)) {
                motorCmd->blta(i, j) = qrParam.tauLimit.max(i, j);
            } else if (motorCmd->blta(i, j) < qrParam.tauLimit.min(i, j)) {
                motorCmd->blta(i, j) = qrParam.tauLimit.min(i, j);
            }
        }
    }
    motorCmdOld_ = *motorCmd;
}

DataCmd CtrlSendData::CheckCmd2(const DataCmd &motorCmd, const DataCmd &motorCmdOld)
{
    DataCmd motorCmdNew = motorCmd;

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 4; j++) {
            // 对Nan的处理非常关键，防止程序崩溃
            if (std::isnan(motorCmdNew.alpha(i, j))) {
                motorCmdNew.alpha(i, j) = motorCmdOld.alpha(i, j);
            }
            if (std::isnan(motorCmdNew.torq(i, j))) {
                motorCmdNew.torq.setZero();  // 对Nan的处理非常关键，防止程序崩溃
            }
            if (std::isnan(motorCmdNew.blta(i, j))) {
                motorCmdNew.blta.setZero();  // 对Nan的处理非常关键，防止程序崩溃
            }

            if (motorCmdNew.alpha(i, j) > qLimitUnitreeMarkerMax_(i, j)) {
                motorCmdNew.alpha(i, j) = qLimitUnitreeMarkerMax_(i, j);
            } else if (motorCmdNew.alpha(i, j) < qLimitUnitreeMarkerMin_(i, j)) {
                motorCmdNew.alpha(i, j) = qLimitUnitreeMarkerMin_(i, j);
            }
        }
    }
    return motorCmdNew;
}

void CtrlSendData::CheckCmd3(DataValue2 *motorCmd)
{
    // 只检查关节的三个电机
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 4; j++) {
            // 对Nan的处理非常关键，防止程序崩溃
            if (std::isnan(motorCmd->alpha(i, j))) {
                motorCmd->alpha(i, j) = motorCmdWheelOld_.alpha(i, j);
            }
            if (std::isnan(motorCmd->torq(i, j))) {
                motorCmd->torq.setZero();  // 对Nan的处理非常关键，防止程序崩溃
            }
            if (std::isnan(motorCmd->blta(i, j))) {
                motorCmd->blta.setZero();  // 对Nan的处理非常关键，防止程序崩溃
            }

            if (motorCmd->alpha(i, j) > qrParam.qLimit.max(i, j)) {
                motorCmd->alpha(i, j) = qrParam.qLimit.max(i, j);
            } else if (motorCmd->alpha(i, j) < qrParam.qLimit.min(i, j)) {
                motorCmd->alpha(i, j) = qrParam.qLimit.min(i, j);
            }
            if (motorCmd->blta(i, j) > qrParam.tauLimit.max(i, j)) {
                motorCmd->blta(i, j) = qrParam.tauLimit.max(i, j);
            } else if (motorCmd->blta(i, j) < qrParam.tauLimit.min(i, j)) {
                motorCmd->blta(i, j) = qrParam.tauLimit.min(i, j);
            }
        }
    }

    for (int i = 0; i < 4; i++) {
        // motorCmd->alpha(3, i) = mathFun_.NormalizeAngle(motorCmd->alpha(3, i));
        if (motorCmd->torq(3, i) > qrParam.qdLimit.max(0, i)) {
            motorCmd->torq(3, i) = qrParam.qdLimit.max(0, i);  // 取第一个关节电机的速度限制值
        } else if (motorCmd->torq(3, i) < qrParam.qdLimit.min(0, i)) {
            motorCmd->torq(3, i) = qrParam.qdLimit.min(0, i);
        }

        if (motorCmd->blta(3, i) > qrParam.tauLimit.max(0, i)) {
            motorCmd->blta(3, i) = qrParam.tauLimit.max(0, i);
        } else if (motorCmd->blta(3, i) < qrParam.tauLimit.min(0, i)) {
            motorCmd->blta(3, i) = qrParam.tauLimit.min(0, i);
        }
    }
    motorCmdWheelOld_ = *motorCmd;
}

void CtrlSendData::CheckCmd4(DataValue2 *motorCmd)
{
    // 只检查关节的三个电机
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 4; j++) {
            // 对Nan的处理非常关键，防止程序崩溃
            if (std::isnan(motorCmd->alpha(i, j))) {
                motorCmd->alpha(i, j) = motorCmdWheelOld_.alpha(i, j);
            }
            if (std::isnan(motorCmd->torq(i, j))) {
                motorCmd->torq.setZero();  // 对Nan的处理非常关键，防止程序崩溃
            }
            if (std::isnan(motorCmd->blta(i, j))) {
                motorCmd->blta.setZero();  // 对Nan的处理非常关键，防止程序崩溃
            }

            if (motorCmd->alpha(i, j) > qLimitUnitreeMarkerMax_(i, j)) {
                motorCmd->alpha(i, j) = qLimitUnitreeMarkerMax_(i, j);
            } else if (motorCmd->alpha(i, j) < qLimitUnitreeMarkerMin_(i, j)) {
                motorCmd->alpha(i, j) = qLimitUnitreeMarkerMin_(i, j);
            }
        }
    }

    for (int i = 0; i < 4; i++) {
        if (motorCmd->torq(3, i) > qrParam.qdLimit.max(0, i)) {
            motorCmd->torq(3, i) = qrParam.qdLimit.max(0, i);  // 取第一个关节电机的速度限制值
        } else if (motorCmd->torq(3, i) < qrParam.qdLimit.min(0, i)) {
            motorCmd->torq(3, i) = qrParam.qdLimit.min(0, i);
        }

        if (motorCmd->blta(3, i) > qrParam.tauLimit.max(0, i)) {
            motorCmd->blta(3, i) = qrParam.tauLimit.max(0, i);
        } else if (motorCmd->blta(3, i) < qrParam.tauLimit.min(0, i)) {
            motorCmd->blta(3, i) = qrParam.tauLimit.min(0, i);
        }
    }
    motorCmdWheelOld_ = *motorCmd;
}

/**
 * @description: 设置标准数据
 * @param {DataCmd} &data 标准数据
 * @return {*}
 */
// void CtrlSendData::SetMotorCmdStandard(const Mat34<double> &data) { motorCmdStandard_.alpha = data; }

void CtrlSendData::SetCxdV3(const Mat34<double> &data)
{
    motorCmdOld_.alpha = data;
    motorCmdMitOld_.alpha = transform.my2mit(data);
    motorCmdWheelOld_.alpha.block(0, 0, 3, 4) = data;  // 同时清除轮足式的腿部关节数据，兼容
}

void CtrlSendData::SetCxdV3(const Mat44<double> &data) { motorCmdWheelOld_.alpha = data; }

/**
 * @description: 发送电机命令，指定q qd tau
 * @param {DataCmd} &motorCmd 电机命令
 * @return {*}
 */
void CtrlSendData::SendMotorCmd(const DataCmd &motorCmd)
{
    CmdVal2 newCmd = motorCmd;

    int curSta = Enum2Num(_curState);
    newCmd.k2 = motorPid[curSta].k2;
    newCmd.k1 = motorPid[curSta].k1;
    newCmd.k3 = motorPid[curSta].k3;

    SendMotorCmd(newCmd);
}

/**
 * @description: 发送电机命令，包含kp kd kff
 * @param &motorCmd
 * @param &kp
 * @param &kd
 * @param &kff
 * @return {}
 */
void CtrlSendData::SendMotorCmd(CmdVal2 motorCmd)
{
    // Algo 坐标系
    CheckCmd(&motorCmd);  // 只对q,tau,检查就行

    CmdVal2 motorCmdMit = MotorCheck(motorCmd);  // todo有问题

    msg::qr::motor_cmd publishCmd;  // 最终发布的电机命令

    if (fromSim_) {
        DataCmd motorCmdUnitree;
        motorCmdUnitree.alpha = transform.qMit2unitree(motorCmdMit.alpha);
        motorCmdUnitree.torq = transform.qdMit2unitree(motorCmdMit.torq);
        motorCmdUnitree.blta = transform.tauMit2unitree(motorCmdMit.blta);
#if 0
        for (int i = 0; i < 4; i++) {
            publishCmd.q_abad[i] = motorCmdUnitree.alpha(0, i);
            publishCmd.q_hip[i] = motorCmdUnitree.alpha(1, i);
            publishCmd.q_knee[i] = motorCmdUnitree.alpha(2, i);

            publishCmd.qd_abad[i] = motorCmdUnitree.torq(0, i);
            publishCmd.qd_hip[i] = motorCmdUnitree.torq(1, i);
            publishCmd.qd_knee[i] = motorCmdUnitree.torq(2, i);

            publishCmd.t_abad[i] = motorCmdUnitree.blta(0, i);
            publishCmd.t_hip[i] = motorCmdUnitree.blta(1, i);
            publishCmd.t_knee[i] = motorCmdUnitree.blta(2, i);

            publishCmd.kp_abad[i] = motorCmd.k2(0, i);
            publishCmd.kp_hip[i] = motorCmd.k2(1, i);
            publishCmd.kp_knee[i] = motorCmd.k2(2, i);

            publishCmd.kd_abad[i] = motorCmd.k1(0, i);
            publishCmd.kd_hip[i] = motorCmd.k1(1, i);
            publishCmd.kd_knee[i] = motorCmd.k1(2, i);
        }
#endif
        for (int leg = 0; leg < 4; leg++) {
            for (int motor = 0; motor < 3; motor++) {
                publishCmd.leg[leg][motor].alpha = motorCmdUnitree.alpha(motor, leg);
                publishCmd.leg[leg][motor].torq = motorCmdUnitree.torq(motor, leg);
                publishCmd.leg[leg][motor].blta = motorCmdUnitree.blta(motor, leg);
                publishCmd.leg[leg][motor].k2 = motorCmd.k2(motor, leg);
                publishCmd.leg[leg][motor].k1 = motorCmd.k1(motor, leg);
            }
        }

    } else {
        // MIT坐标系转真实坐标系
        CmdVal2 motorCmdReal = MotorCheck2(motorCmdMit);

        // 力矩补偿
        // if (qrParam.model.contains(QrModel::linkV2_3)) {
        //     double torqueFixPara;
        //     for (int i = 0; i < 4; i++) {
        //         torqueFixPara = pow(cos(0.5 * M_PI - motorCmd.alpha(2, i) - qrParam.linkAngle), 2);  // 注意，里面使用了Algo坐标系下的值，因为方便直接，与推导相同
        //         motorCmdReal.blta(2, i) /= torqueFixPara;
        //         // cout << "torqueFixPara = " << torqueFixPara << endl;
        //     }
        // }
        // cout << "----------- " << endl;
#if 0
        for (int i = 0; i < 4; i++) {
            publishCmd.q_abad[i] = motorCmdReal.alpha(0, i);
            publishCmd.q_hip[i] = motorCmdReal.alpha(1, i);
            publishCmd.q_knee[i] = motorCmdReal.alpha(2, i);

            publishCmd.qd_abad[i] = motorCmdReal.torq(0, i);
            publishCmd.qd_hip[i] = motorCmdReal.torq(1, i);
            publishCmd.qd_knee[i] = motorCmdReal.torq(2, i);

            publishCmd.kp_abad[i] = motorCmd.k2(0, i);
            publishCmd.kp_hip[i] = motorCmd.k2(1, i);
            publishCmd.kp_knee[i] = motorCmd.k2(2, i);

            publishCmd.kd_abad[i] = motorCmd.k1(0, i);
            publishCmd.kd_hip[i] = motorCmd.k1(1, i);
            publishCmd.kd_knee[i] = motorCmd.k1(2, i);

            publishCmd.t_abad[i] = motorCmd.k3(0, i) * motorCmdReal.blta(0, i);
            publishCmd.t_hip[i] = motorCmd.k3(1, i) * motorCmdReal.blta(1, i);
            publishCmd.t_knee[i] = motorCmd.k3(2, i) * motorCmdReal.blta(2, i);
        }
#endif
        for (int leg = 0; leg < 4; leg++) {
            for (int motor = 0; motor < 3; motor++) {
                publishCmd.leg[leg][motor].alpha = motorCmdReal.alpha(motor, leg);
                publishCmd.leg[leg][motor].torq = motorCmdReal.torq(motor, leg);
                publishCmd.leg[leg][motor].blta = motorCmd.k3(motor, leg) * motorCmdReal.blta(motor, leg);
                publishCmd.leg[leg][motor].k2 = motorCmd.k2(motor, leg);
                publishCmd.leg[leg][motor].k1 = motorCmd.k1(motor, leg);
            }
        }
    }
    // std::cout << "cmd: " << publishCmd.q_hip[0] << ", " << publishCmd.q_hip[1] << std::endl;
    MsgTrySend("qr::motor_cmd", publishCmd);
}
/**
 * @description: 将算法坐标系的电机命令转成MIT坐标系的电机命令
 * @param {DataCmd} &data
 * @return {*}
 */
CmdVal2 CtrlSendData::MotorCheck(const CmdVal2 &motorCmd)
{
    CmdVal2 motorCmdMit;

    motorCmdMit.alpha = transform.my2mit(motorCmd.alpha);

    // motorCmdMit.torq = (motorCmdMit.alpha - motorCmdMitOld_.alpha) / qrParam.dt;  // TODO 算完后再坐标轴转换
    // motorCmdMitOld_.alpha = motorCmdMit.alpha;
    motorCmdMit.torq = motorCmd.torq;
    motorCmdMit.blta = motorCmd.blta;

#if 0
    bool finish = false;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 4; j++) {
            if (fabs(motorCmdMit.torq(i, j)) > 20) {
                LOG_ERROR("qd to big, idx:({},{}), val:{}", j, i, motorCmdMit.torq(i, j));
                motorCmdMit.torq.setZero();
                SetQrEvent(QrEventType::qd_to_big);
                finish = true;
                break;
            }
        }
        if (finish == true) {
            break;
        }
    }
#endif

    return motorCmdMit;
}

/**
 * @description: 将MIT坐标系的电机命令转成真实坐标系的电机命令
 * @param {DataCmd} &motorCmdMit
 * @return {*}
 */
CmdVal2 CtrlSendData::MotorCheck2(const CmdVal2 &motorCmdMit)
{
    DataCmd motorCmdReal;
    motorCmdReal.alpha = transform.qMit2real(motorCmdMit.alpha, transform.my2mit(qrParam.qInit), qrParam.kneeRatio);
    motorCmdReal.torq = transform.qdMit2real(motorCmdMit.torq, qrParam.kneeRatio);
    motorCmdReal.blta = transform.tauMit2real(motorCmdMit.blta, qrParam.kneeRatio);
    return motorCmdReal;
}

/**
 * @description: 宇树坐标系发送，用于AI
 * @param &motorCmd
 * @return {}
 */
void CtrlSendData::SendMotorCmd2(const CmdVal2 &motorCmd)
{
    DataCmd motorCmdUnitree = motorCmd;  // 就是unitree坐标系
    motorCmdUnitree = CheckCmd2(motorCmdUnitree, motorCmdUnitreeOld_);
    msg::qr::motor_cmd publishCmd;  // 最终发布的电机命令
    if (fromSim_) {
        // 仿真时，本来就是unitree坐标系
#if 0
        for (int i = 0; i < 4; i++) {
            publishCmd.q_abad[i] = motorCmdUnitree.alpha(0, i);
            publishCmd.q_hip[i] = motorCmdUnitree.alpha(1, i);
            publishCmd.q_knee[i] = motorCmdUnitree.alpha(2, i);

            publishCmd.qd_abad[i] = motorCmdUnitree.torq(0, i);
            publishCmd.qd_hip[i] = motorCmdUnitree.torq(1, i);
            publishCmd.qd_knee[i] = motorCmdUnitree.torq(2, i);

            publishCmd.t_abad[i] = motorCmdUnitree.blta(0, i);
            publishCmd.t_hip[i] = motorCmdUnitree.blta(1, i);
            publishCmd.t_knee[i] = motorCmdUnitree.blta(2, i);

            publishCmd.kp_abad[i] = motorCmd.k2(0, i);
            publishCmd.kp_hip[i] = motorCmd.k2(1, i);
            publishCmd.kp_knee[i] = motorCmd.k2(2, i);

            publishCmd.kd_abad[i] = motorCmd.k1(0, i);
            publishCmd.kd_hip[i] = motorCmd.k1(1, i);
            publishCmd.kd_knee[i] = motorCmd.k1(2, i);
            // cout << "kp_hip[i]" << motorCmd.k2(1, i) << endl;
        }
#endif
        for (int leg = 0; leg < 4; leg++) {
            for (int motor = 0; motor < 3; motor++) {
                publishCmd.leg[leg][motor].alpha = motorCmdUnitree.alpha(motor, leg);
                publishCmd.leg[leg][motor].torq = motorCmdUnitree.torq(motor, leg);
                publishCmd.leg[leg][motor].blta = motorCmdUnitree.blta(motor, leg);
                publishCmd.leg[leg][motor].k2 = motorCmd.k2(motor, leg);
                publishCmd.leg[leg][motor].k1 = motorCmd.k1(motor, leg);
            }
        }

    } else {
        // 宇树转实物
        DataCmd motorCmdReal;
        // cout << "unitree2real" << endl;
        motorCmdReal.alpha = transform.qUnitree2real(motorCmdUnitree.alpha, qrParam.qInit, qrParam.kneeRatio);
        motorCmdReal.torq = transform.qdUnitree2real(motorCmdUnitree.torq, qrParam.kneeRatio);
        motorCmdReal.blta = transform.tauUnitree2real(motorCmdUnitree.blta, qrParam.kneeRatio);
#if 0
        for (int i = 0; i < 4; i++) {
            publishCmd.q_abad[i] = motorCmdReal.alpha(0, i);
            publishCmd.q_hip[i] = motorCmdReal.alpha(1, i);
            publishCmd.q_knee[i] = motorCmdReal.alpha(2, i);

            publishCmd.qd_abad[i] = motorCmdReal.torq(0, i);
            publishCmd.qd_hip[i] = motorCmdReal.torq(1, i);
            publishCmd.qd_knee[i] = motorCmdReal.torq(2, i);

            publishCmd.kp_abad[i] = motorCmd.k2(0, i);
            publishCmd.kp_hip[i] = motorCmd.k2(1, i);
            publishCmd.kp_knee[i] = motorCmd.k2(2, i);

            publishCmd.kd_abad[i] = motorCmd.k1(0, i);
            publishCmd.kd_hip[i] = motorCmd.k1(1, i);
            publishCmd.kd_knee[i] = motorCmd.k1(2, i);

            publishCmd.t_abad[i] = motorCmdReal.blta(0, i);
            publishCmd.t_hip[i] = motorCmdReal.blta(1, i);
            publishCmd.t_knee[i] = motorCmdReal.blta(2, i);
        }
#endif
        for (int leg = 0; leg < 4; leg++) {
            for (int motor = 0; motor < 3; motor++) {
                publishCmd.leg[leg][motor].alpha = motorCmdReal.alpha(motor, leg);
                publishCmd.leg[leg][motor].torq = motorCmdReal.torq(motor, leg);
                publishCmd.leg[leg][motor].blta = motorCmdReal.blta(motor, leg);
                publishCmd.leg[leg][motor].k2 = motorCmd.k2(motor, leg);
                publishCmd.leg[leg][motor].k1 = motorCmd.k1(motor, leg);
            }
        }
    }
    // cout << "----------------------------------------" << endl;
    MsgTrySend("qr::motor_cmd", publishCmd);
}

/**
 * @description: 发送轮足式数据，带PID todo 检查坐标轴转换
 * @param motorCmd
 * @return {}
 */
void CtrlSendData::SendMotorCmd2(DataValue2 motorCmd)
{
    CheckCmd4(&motorCmd);

    // std::ostringstream oss;
    // oss << "s ";
    // for (int leg = 0; leg < 4; leg++) {
    //     for (int motor = 0; motor < 3; motor++) {
    //         oss << motorCmd.alpha(motor, leg) << " ";
    //     }
    // }
    // std::cout << oss.str() << std::endl;

    msg::wheel::motor_cmd publishCmd;  // 带发送的命令

    if (fromSim_) {
        for (int leg = 0; leg < 4; leg++) {
            for (int motor = 0; motor < 4; motor++) {
                publishCmd.leg[leg][motor].alpha = motorCmd.alpha(motor, leg);
                publishCmd.leg[leg][motor].torq = motorCmd.torq(motor, leg);
                publishCmd.leg[leg][motor].blta = motorCmd.blta(motor, leg);
            }
        }
    } else {
        CmdVal4 motorCmdReal;
        motorCmdReal.alpha.block(0, 0, 3, 4) = transform.qUnitree2real(motorCmd.alpha.block(0, 0, 3, 4), qrParam.qInit, qrParam.kneeRatio);
        motorCmdReal.torq.block(0, 0, 3, 4) = transform.qdUnitree2real(motorCmd.torq.block(0, 0, 3, 4), qrParam.kneeRatio);
        motorCmdReal.blta.block(0, 0, 3, 4) = transform.tauUnitree2real(motorCmd.blta.block(0, 0, 3, 4), qrParam.kneeRatio);

        for (int leg = 0; leg < 4; leg++) {
            for (int motor = 0; motor < 3; motor++) {
                publishCmd.leg[leg][motor].alpha = motorCmdReal.alpha(motor, leg);
                publishCmd.leg[leg][motor].torq = motorCmdReal.torq(motor, leg);
                publishCmd.leg[leg][motor].blta = motorCmdReal.blta(motor, leg);
            }

            publishCmd.leg[leg][3].alpha = motorCmd.alpha(3, leg);
            publishCmd.leg[leg][3].torq = motorCmd.torq(3, leg);
            publishCmd.leg[leg][3].blta = motorCmd.blta(3, leg);
        }
        publishCmd.leg[0][3].alpha *= -1;
        publishCmd.leg[2][3].alpha *= -1;
        publishCmd.leg[0][3].torq *= -1;
        publishCmd.leg[2][3].torq *= -1;
        publishCmd.leg[0][3].blta *= -1;
        publishCmd.leg[2][3].blta *= -1;
    }

    for (int leg = 0; leg < 4; leg++) {
        for (int motor = 0; motor < 4; motor++) {
            publishCmd.leg[leg][motor].blta *= motorCmd.k3(motor, leg);
            publishCmd.leg[leg][motor].k2 = motorCmd.k2(motor, leg);
            publishCmd.leg[leg][motor].k1 = motorCmd.k1(motor, leg);
        }
    }

    // cout << "####################" << endl;
    // cout << publishCmd.leg[0][3].torq << endl;
    // cout << publishCmd.leg[0][3].k1 << endl;
    // cout << "-----------" << endl;
    // cout << publishCmd.leg[1][3].torq << endl;
    // cout << publishCmd.leg[1][3].k1 << endl;
    // cout << "-----------" << endl;
    // cout << publishCmd.leg[2][3].torq << endl;
    // cout << publishCmd.leg[2][3].k1 << endl;
    // cout << "-----------" << endl;
    // cout << publishCmd.leg[3][3].torq << endl;
    // cout << publishCmd.leg[3][3].k1 << endl;

    MsgTrySend("wheel::motor_cmd", publishCmd);
}

void CtrlSendData::SendMotorCmdFixed(DataValue2 motorCmd)
{
    msg::wheel::motor_cmd publishCmd;  // 带发送的命令

    if (fromSim_) {
        // 仿真时，本来就是unitree坐标系
        for (int leg = 0; leg < 4; leg++) {
            for (int motor = 0; motor < 4; motor++) {
                publishCmd.leg[leg][motor].alpha = motorCmd.alpha(motor, leg);
                publishCmd.leg[leg][motor].torq = motorCmd.torq(motor, leg);
                publishCmd.leg[leg][motor].blta = motorCmd.blta(motor, leg);
            }
        }
    } else {
        // 宇树转实物
        CmdVal4 motorCmdReal;

        motorCmdReal.alpha.block(0, 0, 3, 4) = transform.qUnitree2Real(motorCmd.alpha.block(0, 0, 3, 4), qrParam.qInitUnitree, qrParam.kneeRatio);
        motorCmdReal.torq.block(0, 0, 3, 4) = transform.qdUnitree2Real(motorCmd.torq.block(0, 0, 3, 4), qrParam.kneeRatio);
        motorCmdReal.blta.block(0, 0, 3, 4) = transform.tauUnitree2Real(motorCmd.blta.block(0, 0, 3, 4), qrParam.kneeRatio);

        for (int leg = 0; leg < 4; leg++) {
            for (int motor = 0; motor < 3; motor++) {
                publishCmd.leg[leg][motor].alpha = motorCmdReal.alpha(motor, leg);
                publishCmd.leg[leg][motor].torq = motorCmdReal.torq(motor, leg);
                publishCmd.leg[leg][motor].blta = motorCmdReal.blta(motor, leg);
            }
            publishCmd.leg[leg][3].alpha = motorCmd.alpha(3, leg);
            publishCmd.leg[leg][3].torq = motorCmd.torq(3, leg);
            publishCmd.leg[leg][3].blta = motorCmd.blta(3, leg);
        }
        publishCmd.leg[0][3].alpha *= -1;
        publishCmd.leg[2][3].alpha *= -1;
        publishCmd.leg[0][3].torq *= -1;
        publishCmd.leg[2][3].torq *= -1;
        publishCmd.leg[0][3].blta *= -1;
        publishCmd.leg[2][3].blta *= -1;
    }

    // 增加kp kd kf的影响
    for (int leg = 0; leg < 4; leg++) {
        for (int motor = 0; motor < 4; motor++) {
            publishCmd.leg[leg][motor].blta *= motorCmd.k3(motor, leg);
            publishCmd.leg[leg][motor].k2 = motorCmd.k2(motor, leg);
            publishCmd.leg[leg][motor].k1 = motorCmd.k1(motor, leg);
        }
    }

    MsgTrySend("wheel::motor_cmd", publishCmd);
}


/**
 * @description: 发送轮足式数据，带PID todo 检查坐标轴转换
 * @param motorCmd
 * @return {}
 */
void CtrlSendData::SendMotorCmd(DataValue2 motorCmd)
{
    CheckCmd3(&motorCmd);
    // std:cout << motorCmd.alpha.block(2, 0, 1, 4) << std::endl;

    // for(int i = 0; i < 4; i++)
    // {
    //     // motorCmd.alpha(0, i) = 0.1 * i;
    //     // motorCmd.alpha(1, i) = 0.1 * i;
    //     // motorCmd.alpha(2, i) = 1.57 + i * 0.1;
    //     motorCmd.torq(3, i) = -1.0;
    //     // for(int j = 0; j < 4; j++)
    //     // {
    //     //     std::cout << "[leg, motor]: " << i << " " << j << " --> "  << publishCmd.leg[i][j].alpha << std::endl;
    //     // }
    // }

    // TODO: 后四个关节没处理，第四行没处理
    CmdVal2 tempCmd;  // 3 * 4
    tempCmd.alpha = motorCmd.alpha.block(0, 0, 3, 4);
    tempCmd.torq = motorCmd.torq.block(0, 0, 3, 4);
    tempCmd.blta = motorCmd.blta.block(0, 0, 3, 4);
    CmdVal2 tempMotorCmdMit = MotorCheck(tempCmd);  // 关节速度在里面求

    msg::wheel::motor_cmd publishCmd;  // 带发送的命令

    if (fromSim_) {
        DataCmd motorCmdUnitree;
        motorCmdUnitree.alpha = transform.qMit2unitree(tempMotorCmdMit.alpha);
        motorCmdUnitree.torq = transform.qdMit2unitree(tempMotorCmdMit.torq);
        motorCmdUnitree.blta = transform.tauMit2unitree(tempMotorCmdMit.blta);

        for (int leg = 0; leg < 4; leg++) {
            for (int motor = 0; motor < 3; motor++) {
                publishCmd.leg[leg][motor].alpha = motorCmdUnitree.alpha(motor, leg);
                publishCmd.leg[leg][motor].torq = motorCmdUnitree.torq(motor, leg);
                publishCmd.leg[leg][motor].blta = motorCmdUnitree.blta(motor, leg);
            }
            publishCmd.leg[leg][3].alpha = motorCmd.alpha(3, leg);
            publishCmd.leg[leg][3].torq = motorCmd.torq(3, leg);
            publishCmd.leg[leg][3].blta = motorCmd.blta(3, leg);
        }

        // std::cout << "----------------" << std::endl;
        // for(int i = 0; i < 4; i++)
        // {
        //     // publishCmd.leg[i][0].alpha = 0;
        //     // publishCmd.leg[i][1].alpha = 0;
        //     // publishCmd.leg[i][2].alpha = -1.57;
        //     // publishCmd.leg[i][3].torq = 0.1;
        //     // for(int j = 0; j < 4; j++)
        //     // {
        //     //     std::cout << "[leg, motor]: " << i << " " << j << " --> "  << publishCmd.leg[i][j].alpha << std::endl;
        //     // }
        //     std::cout << "[leg, motor]: " << i << " " << 2 << " --> "  << publishCmd.leg[i][2].alpha << std::endl;
        // }

    } else {
        CmdVal2 motorCmdReal = MotorCheck2(tempMotorCmdMit);

        // 宇树转实物
        // CmdVal4 motorCmdReal;
        // motorCmdReal.alpha.block(0, 0, 3, 4) = transform.qUnitree2real(motorCmd.alpha.block(0, 0, 3, 4), qrParam.qInit, qrParam.kneeRatio);
        // motorCmdReal.torq.block(0, 0, 3, 4) = transform.qdUnitree2real(motorCmd.torq.block(0, 0, 3, 4), qrParam.kneeRatio);
        // motorCmdReal.blta.block(0, 0, 3, 4) = transform.tauUnitree2real(motorCmd.blta.block(0, 0, 3, 4), qrParam.kneeRatio);

        for (int leg = 0; leg < 4; leg++) {
            for (int motor = 0; motor < 3; motor++) {
                publishCmd.leg[leg][motor].alpha = motorCmdReal.alpha(motor, leg);
                publishCmd.leg[leg][motor].torq = motorCmdReal.torq(motor, leg);
                publishCmd.leg[leg][motor].blta = motorCmdReal.blta(motor, leg);
            }
            // motorCmdReal 3x4,  motorCmd 4x4
            publishCmd.leg[leg][3].alpha = motorCmd.alpha(3, leg);
            publishCmd.leg[leg][3].torq = motorCmd.torq(3, leg);
            publishCmd.leg[leg][3].blta = motorCmd.blta(3, leg);
        }

        // 此处实物轮部电机为右手定则
        publishCmd.leg[0][3].alpha *= -1;
        publishCmd.leg[2][3].alpha *= -1;
        publishCmd.leg[0][3].torq *= -1;
        publishCmd.leg[2][3].torq *= -1;
        publishCmd.leg[0][3].blta *= -1;
        publishCmd.leg[2][3].blta *= -1;
    }

    // 增加kp kd kf的影响
    for (int leg = 0; leg < 4; leg++) {
        for (int motor = 0; motor < 4; motor++) {
            publishCmd.leg[leg][motor].blta *= motorCmd.k3(motor, leg);
            publishCmd.leg[leg][motor].k2 = motorCmd.k2(motor, leg);
            publishCmd.leg[leg][motor].k1 = motorCmd.k1(motor, leg);
        }
    }

    // for (int leg = 0; leg < 4; leg++) {
    //     for (int motor = 0; motor < 3; motor++) {
    //         publishCmd.leg[leg][motor].blta *= 0;
    //         publishCmd.leg[leg][motor].k2 = 50;
    //         publishCmd.leg[leg][motor].k1 = 3;
    //     }
    //     publishCmd.leg[leg][3].blta *= 0;
    //     publishCmd.leg[leg][3].k2 = 0;
    //     publishCmd.leg[leg][3].k1 = 2;

    // }

    // for (int leg = 0; leg < 4; leg++) {
    //     for (int motor = 0; motor < 4; motor++) {
    //         publishCmd.leg[leg][motor].alpha = 0;
    //         publishCmd.leg[leg][motor].torq = 10;
    //         publishCmd.leg[leg][motor].blta = 0;
    //         publishCmd.leg[leg][motor].k2 = 0;
    //         publishCmd.leg[leg][motor].k1 = 0.4;
    //     }
    // }

    // cout << " publishCmd.leg33: " << publishCmd.leg[3][3].torq << endl;

    MsgTrySend("wheel::motor_cmd", publishCmd);
}

/**
 * @description: 发送轮足式的控制，不带PID
 * @param motorCmd
 * @return {}
 */
void CtrlSendData::SendMotorCmd(const CmdVal4 &motorCmd)
{
    DataValue2 motorCmdPid = motorCmd;  // 此处走拷贝构造函数，不是operator=

    // 添加pid
    motorCmdPid.k2.fill(100);
    motorCmdPid.k1.fill(1);
    motorCmdPid.k3.fill(1);

    SendMotorCmd(motorCmdPid);
}

/**
 * @description: 广播期望的姿态指令和实际返回的姿态指令
 * @param {Vec3<double>} &rpyDesire 期望的rpy指令
 * @param {Vec3<double>} &rpyFilter 过滤后的rpy返回
 * @return {*}
 */
void PublishBodyDesire(const Vec3<double> &rpyDesire, const Vec3<double> &wDesire, const Vec3<double> &aDesire, const Vec4<double> &quat)
{
    msg::imu_data imuDesire;
    for (int i = 0; i < 3; i++) {
        imuDesire.ang[i] = rpyDesire(i);
        imuDesire.gyro[i] = wDesire(i);
        imuDesire.acc[i] = aDesire(i);
        imuDesire.quat[i] = quat(i);
    }
    imuDesire.quat[3] = quat(3);
    MsgTrySend("qr::rpyCmd", imuDesire);
}

void PublishBodyCurrent(const Vec3<double> &rpyFilter, const Vec3<double> &wFilter, const Vec3<double> &aFilter, const Vec4<double> &quat)
{
    msg::imu_data imuFilter;
    for (int i = 0; i < 3; i++) {
        imuFilter.ang[i] = rpyFilter(i);
        imuFilter.gyro[i] = wFilter(i);
        imuFilter.acc[i] = aFilter(i);
        imuFilter.quat[i] = quat(i);
    }
    imuFilter.quat[3] = quat(3);
    MsgTrySend("qr::rpyFilter", imuFilter);
}
/**
 * @description: 广播需要被监控的数据，把PublishInterestData分成三个三个函数
 * @param {vector<Vec3<double>>} &data 最多六个图表，具体意义见下
 * @return {*}
 */
void PublishInterestDataC(const Vec3<double> &c, const Vec3<double> &rc)
{
    msg::watch_data_c debug3;

    for (int i = 0; i < 3; i++) {
        debug3.c[i] = c(i);    // 存放在a页的三个图表的蓝色线
        debug3.rc[i] = rc(i);  // 存放在a页的三个图表的红色线
    }
    // MsgSend msgDebugData("debugDataC");
    // msgDebugData.TrySend(debug3, 50);

    MsgTrySend("qr::debugDataC", debug3);
}
/**
 * @description: 广播需要被监控的数据，把PublishInterestData分成三个三个函数
 * @param {vector<Vec3<double>>} &data 最多六个图表，具体意义见下
 * @return {*}
 */
void PublishInterestDataB(const Vec3<double> &b, const Vec3<double> &rb)
{
    msg::watch_data_b debug2;

    for (int i = 0; i < 3; i++) {
        debug2.b[i] = b(i);    // 存放在a页的三个图表的蓝色线
        debug2.rb[i] = rb(i);  // 存放在a页的三个图表的红色线
    }

    MsgTrySend("qr::debugDataB", debug2);
}
/**
 * @description: 广播需要被监控的数据，把PublishInterestData分成三个三个函数
 * @param {vector<Vec3<double>>} &data 最多六个图表，具体意义见下
 * @return {*}
 */
void PublishInterestDataA(const Vec3<double> &a, const Vec3<double> &ra)
{
    msg::watch_data_a debug1;

    for (int i = 0; i < 3; i++) {
        debug1.a[i] = a(i);    // 存放在a页的三个图表的蓝色线
        debug1.ra[i] = ra(i);  // 存放在a页的三个图表的红色线
    }

    MsgTrySend("qr::debugDataA", debug1);
}
/**
 * @description: 广播需要被监控的数据
 * @param {vector<Vec3<double>>} &data 最多六个图表，具体意义见下
 * @return {*}
 */
void PublishInterestData(const Vec3<double> &a, const Vec3<double> &ra, const Vec3<double> &b, const Vec3<double> &rb, const Vec3<double> &c, const Vec3<double> &rc)
{
    msg::watch_data debug;

    for (int i = 0; i < 3; i++) {
        debug.a[i] = a(i);    // 存放在a页的三个图表的蓝色线
        debug.ra[i] = ra(i);  // 存放在a页的三个图表的红色线
        debug.b[i] = b(i);    // 存放在b页的三个图表的蓝色线
        debug.rb[i] = rb(i);  // 存放在b页的三个图表的红色线
        debug.c[i] = c(i);    // 存放在c页的三个图表的蓝色线
        debug.rc[i] = rc(i);  // 存放在c页的三个图表的红色线
    }

    MsgTrySend("qr::debugData", debug);
}

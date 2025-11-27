

#include "ctrlRecvData.hpp"

#include "baseline.hpp"
#include "ctrlSendData.hpp"
#include "mathTools.hpp"

using namespace ::std;

CtrlRecvData::CtrlRecvData(bool fromSim) : fromSim_(fromSim)
{
    // fromSim_ = false;  // 永远真实，适配gazebo模型调整

    TimerTools::SleepForMs(10);

}

/**
 * @description: 载入外部原始数据
 * @param {*}
 * @return {*}
 */
void CtrlRecvData::LoadData()
{
    // 接收imu数据
    if (auto ret = MsgTryRecv<msg::imu_data>("qr::imuData", this)) {
        rawImuData_ = ret.value();
    }

    // 接收电机数据
    if (auto ret = MsgTryRecv<msg::qr::motor_ret>("qr::motor_ret", this)) {
        rawMotionData_ = ret.value();
    }

    if (auto ret = MsgTryRecv<msg::wheel::motor_ret>("wheel::motor_ret", this)) {
        rawMotorWheelData_ = ret.value();
    }

    Deal();
}

/**
 * @description: 对原始数据进行必要的处理
 * @param {*}
 * @return {*}
 */
void CtrlRecvData::Deal()
{
    // 电机原始数据处理
    // for (int i = 0; i < 4; i++) {
    //     std::cout << "q_abad" << i << ":" << rawMotionData_.q_abad[i] << std::endl;
    //     std::cout << "q_hip" << i << ":" << rawMotionData_.q_hip[i] << std::endl;
    //     std::cout << "q_knee" << i << ":" << rawMotionData_.q_knee[i] << std::endl;
    // }
    // rawMotionData_  MIT坐标系
    motorData_ = DealMotionData(rawMotionData_, motorData3_);  // 根据仿真还是实物进行不同坐标系变换操作
    // motorData  sim：始终是My坐标系

    // std::cout << "q_hip_deal" << motorData_.alpha.row(0) << std::endl;

    // imu原始数据处理
    imuData_.LoadData(rawImuData_);

    // 轮足数据处理
    if (qrParam.model.contains(QrModel::linkV2_3_w) || qrParam.model.contains(QrModel::gazebo_w)) {
        DealLegWheelMotorData(rawMotorWheelData_);
    }
}

DataCmd CtrlRecvData::DealMotionData(const msg::qr::motor_ret& data, DataCmd& oldData)
{
    DataCmd motorData;
    DataCmd motorDataReal;
    DataCmd motorDataMit;
    DataCmd motorDataUnitree;

    if (fromSim_) {
#if 0
        for (int i = 0; i < 4; i++) {
            motorDataUnitree.alpha(0, i) = data.q_abad[i];
            motorDataUnitree.torq(0, i) = data.qd_abad[i];
            motorDataUnitree.blta(0, i) = data.t_abad[i];
            motorDataUnitree.alpha(1, i) = data.q_hip[i];
            motorDataUnitree.torq(1, i) = data.qd_hip[i];
            motorDataUnitree.blta(1, i) = data.t_hip[i];
            motorDataUnitree.alpha(2, i) = data.q_knee[i];
            motorDataUnitree.torq(2, i) = data.qd_knee[i];
            motorDataUnitree.blta(2, i) = data.t_knee[i];
        }
#endif
        for (int leg = 0; leg < 4; leg++) {
            for (int motor = 0; motor < 3; motor++) {
                motorDataUnitree.alpha(motor, leg) = data.leg[leg][motor].alpha;
                motorDataUnitree.torq(motor, leg) = data.leg[leg][motor].torq;
                motorDataUnitree.blta(motor, leg) = data.leg[leg][motor].blta;
            }
        }

        motorData2_ = motorDataUnitree;

        motorDataMit.alpha = transform_.qUnitree2mit(motorDataUnitree.alpha);
        motorDataMit.torq = transform_.qUnitree2mit(motorDataUnitree.torq);
        motorDataMit.blta = transform_.qUnitree2mit(motorDataUnitree.blta);

        motorData.alpha = transform_.mit2my(motorDataMit.alpha);
        motorData.torq = motorDataMit.torq;
        motorData.blta = motorDataMit.blta;

    } else {
#if 0
        for (int i = 0; i < 4; i++) {
            motorDataReal.alpha(0, i) = data.q_abad[i];
            motorDataReal.torq(0, i) = data.qd_abad[i];
            motorDataReal.blta(0, i) = data.t_abad[i];

            motorDataReal.alpha(1, i) = data.q_hip[i];
            motorDataReal.torq(1, i) = data.qd_hip[i];
            motorDataReal.blta(1, i) = data.t_hip[i];

            motorDataReal.alpha(2, i) = data.q_knee[i];
            motorDataReal.torq(2, i) = data.qd_knee[i];
            motorDataReal.blta(2, i) = data.t_knee[i];
        }
#endif
        for (int leg = 0; leg < 4; leg++) {
            for (int motor = 0; motor < 3; motor++) {
                motorDataReal.alpha(motor, leg) = data.leg[leg][motor].alpha;
                motorDataReal.torq(motor, leg) = data.leg[leg][motor].torq;
                motorDataReal.blta(motor, leg) = data.leg[leg][motor].blta;
            }
        }

        motorData2_.alpha = transform_.qReal2unitree(motorDataReal.alpha, qrParam.qInit, qrParam.kneeRatio);
        motorData2_.torq = transform_.qdReal2unitree(motorDataReal.torq, qrParam.kneeRatio);
        motorData2_.blta = transform_.tauReal2unitree(motorDataReal.blta, qrParam.kneeRatio);

        motorDataMit.alpha = transform_.qReal2mit(motorDataReal.alpha, transform_.my2mit(qrParam.qInit), qrParam.kneeRatio);
        motorData.alpha = transform_.mit2my(motorDataMit.alpha);
        motorDataMit.torq = transform_.qdReal2mit(motorDataReal.torq, qrParam.kneeRatio);
        motorData.torq = motorDataMit.torq;
        motorDataMit.blta = transform_.tauReal2mit(motorDataReal.blta, qrParam.kneeRatio);
        motorData.blta = motorDataMit.blta;
    }

    motorData = CheckData(motorData, oldData);
    oldData = motorData;  // 保存旧数据
    return motorData;
};

DataCmd CtrlRecvData::CheckData(const DataCmd& motorData, const DataCmd& motorDataOld)
{
    DataCmd motorDataNew = motorData;

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 4; j++) {
            // 对Nan的处理非常关键，防止程序崩溃
            if (std::isnan(motorDataNew.alpha(i, j))) {
                motorDataNew.alpha(i, j) = motorDataOld.alpha(i, j);
            }
            if (std::isnan(motorDataNew.torq(i, j))) {
                motorDataNew.torq.setZero();  // 对Nan的处理非常关键，防止程序崩溃
            }
            if (std::isnan(motorDataNew.blta(i, j))) {
                motorDataNew.blta.setZero();  // 对Nan的处理非常关键，防止程序崩溃
            }

            if (motorDataNew.alpha(i, j) > qrParam.qLimit.max(i, j)) {
                motorDataNew.alpha(i, j) = qrParam.qLimit.max(i, j);
            } else if (motorDataNew.alpha(i, j) < qrParam.qLimit.min(i, j)) {
                motorDataNew.alpha(i, j) = qrParam.qLimit.min(i, j);
            }
            if (motorDataNew.blta(i, j) > qrParam.tauLimit.max(i, j)) {
                motorDataNew.blta(i, j) = qrParam.tauLimit.max(i, j);
            } else if (motorDataNew.blta(i, j) < qrParam.tauLimit.min(i, j)) {
                motorDataNew.blta(i, j) = qrParam.tauLimit.min(i, j);
            }
        }
    }
    return motorDataNew;
}

void CtrlRecvData::DealLegWheelMotorData(const msg::wheel::motor_ret& rawData)
{
    if (fromSim_) {
        // TODO: urdf轮子的转动惯量可能不准确，进行校对
        // 仿真的话，传入的数据是unitree坐标系，因此不做任何转换
        for (int leg = 0; leg < 4; leg++) {
            for (int motor = 0; motor < 4; motor++) {
                motorDataWheel2_.alpha(motor, leg) = rawData.leg[leg][motor].alpha;
                motorDataWheel2_.torq(motor, leg) = rawData.leg[leg][motor].torq;
                motorDataWheel2_.blta(motor, leg) = rawData.leg[leg][motor].blta;
            }
        }
    } else {
        // 实物传入的是real坐标系，需要转成unitree坐标系
        CmdVal4 motorDataReal;  // 临时用于坐标轴转换
        for (int leg = 0; leg < 4; leg++) {
            for (int motor = 0; motor < 4; motor++) {
                motorDataReal.alpha(motor, leg) = rawData.leg[leg][motor].alpha;
                motorDataReal.torq(motor, leg) = rawData.leg[leg][motor].torq;
                motorDataReal.blta(motor, leg) = rawData.leg[leg][motor].blta;
            }
        }

        // 转成unitree坐标系，填充到前面mat34
        motorDataWheel2_.alpha.block(0, 0, 3, 4) = transform_.qReal2Unitree(motorDataReal.alpha.block(0, 0, 3, 4), qrParam.qInitUnitree, qrParam.kneeRatio);
        motorDataWheel2_.torq.block(0, 0, 3, 4) = transform_.qdReal2Unitree(motorDataReal.torq.block(0, 0, 3, 4), qrParam.kneeRatio);
        motorDataWheel2_.blta.block(0, 0, 3, 4) = transform_.tauReal2Unitree(motorDataReal.blta.block(0, 0, 3, 4), qrParam.kneeRatio);

        for (int i = 0; i < 4; i++) {

            motorDataWheel2_.alpha(3, i) = motorDataReal.alpha(3, i);
            motorDataWheel2_.torq(3, i) = motorDataReal.torq(3, i);
            motorDataWheel2_.blta(3, i) = motorDataReal.blta(3, i);
        }

        motorDataWheel2_.alpha(3, 0) *= -1;
        motorDataWheel2_.alpha(3, 2) *= -1;
        motorDataWheel2_.torq(3, 0) *= -1;
        motorDataWheel2_.torq(3, 2) *= -1;
        motorDataWheel2_.blta(3, 0) *= -1;
        motorDataWheel2_.blta(3, 2) *= -1;
    }
    motorDataWheel_ = MotorDataUnitree2Algo(motorDataWheel2_);
    
    motorData_.alpha = motorDataWheel_.alpha.block(0, 0, 3, 4);
    motorData_.torq = motorDataWheel_.torq.block(0, 0, 3, 4);
    motorData_.blta = motorDataWheel_.blta.block(0, 0, 3, 4);
    motorData2_.alpha = motorDataWheel2_.alpha.block(0, 0, 3, 4);
    motorData2_.torq = motorDataWheel2_.torq.block(0, 0, 3, 4);
    motorData2_.blta = motorDataWheel2_.blta.block(0, 0, 3, 4);
}

void CtrlRecvData::CheckData(CmdVal4* motorData, CmdVal4* motorDataOld)
{
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 4; j++) {
            // 对Nan的处理非常关键，防止程序崩溃
            if (std::isnan(motorData->alpha(i, j))) {
                motorData->alpha(i, j) = motorDataOld->alpha(i, j);
            }
            if (std::isnan(motorData->torq(i, j))) {
                motorData->torq.setZero();  // 对Nan的处理非常关键，防止程序崩溃
            }
            if (std::isnan(motorData->blta(i, j))) {
                motorData->blta.setZero();  // 对Nan的处理非常关键，防止程序崩溃
            }
            if (motorData->alpha(i, j) > qrParam.qLimit.max(i, j)) {
                motorData->alpha(i, j) = qrParam.qLimit.max(i, j);
            } else if (motorData->alpha(i, j) < qrParam.qLimit.min(i, j)) {
                motorData->alpha(i, j) = qrParam.qLimit.min(i, j);
            }
            if (motorData->blta(i, j) > qrParam.tauLimit.max(i, j)) {
                motorData->blta(i, j) = qrParam.tauLimit.max(i, j);
            } else if (motorData->blta(i, j) < qrParam.tauLimit.min(i, j)) {
                motorData->blta(i, j) = qrParam.tauLimit.min(i, j);
            }
        }
    }

    for (int i = 0; i < 4; i++) {
        motorData->alpha(3, i) = mathFun_.NormalizeAngle(motorData->alpha(3, i));
        if (motorData->torq(3, i) > qrParam.qdLimit.max(0, i)) {
            motorData->torq(3, i) = qrParam.qdLimit.max(0, i);  // 取第一个关节电机的速度限制值
        } else if (motorData->torq(3, i) < qrParam.qdLimit.min(0, i)) {
            motorData->torq(3, i) = qrParam.qdLimit.min(0, i);
        }

        if (motorData->blta(3, i) > qrParam.tauLimit.max(0, i)) {
            motorData->blta(3, i) = qrParam.tauLimit.max(0, i);
        } else if (motorData->blta(3, i) < qrParam.tauLimit.min(0, i)) {
            motorData->blta(3, i) = qrParam.tauLimit.min(0, i);
        }
    }
    // cout << "motorData->alpha: " << fmod(motorData->alpha(3, 1), 2 * 3.1415926)<< endl;
    *motorDataOld = *motorData;  // 保存处理后的数据到旧数据
}

CmdVal4 CtrlRecvData::MotorDataUnitree2Algo(const CmdVal4 &wheelMotorDataUnitree)
{
    CmdVal4 tempData = wheelMotorDataUnitree;
    for(int i = 0; i < 4; i++)
    {
        tempData.alpha(1, i) = M_PI_2 - wheelMotorDataUnitree.alpha(1, i);
    }
    tempData.alpha.row(2) *= -1;
    tempData.torq.row(1) *= -1;
    tempData.torq.row(2) *= -1;
    tempData.blta.row(1) *= -1;
    tempData.blta.row(2) *= -1;

    return tempData;
}
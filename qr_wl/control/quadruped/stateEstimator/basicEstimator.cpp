#include "basicEstimator.hpp"

#include "baseline.hpp"
#include "ctrlRecvData.hpp"
#include "ctrlSendData.hpp"
#include "orientation_tools.h"

using namespace std;
using namespace ori;
using namespace Eigen;

BasicEstimator::BasicEstimator() { Init(); }

void BasicEstimator::Init() { vCom100.setZero(); }

/**
 * @description: 数据载入
 * @param imuData
 * @return {}
 */
void BasicEstimator::LoadData(const ImuData& imuData) { imuData_ = imuData; }
void BasicEstimator::LoadData(const DataCmd& motorData) { motorData_ = motorData; }
void BasicEstimator::LoadData(const JpDataOutput& jpData) { jpData_ = jpData; }

/**
 * @description: 结果获取
 * @param output 本估计器会重置所有的值
 * @return {}
 */
const StateEstimatorOutputData& BasicEstimator::GetResult() const { return output_; }

void BasicEstimator::Run(AlgoState sta)
{
    output_.quat = imuData_.quat;

    output_.fFoot = forwardDynamics(motorData_.blta, motorData_.alpha);

    Vec3<double> aTemp = imuData_.a;
    Vec3<double> tempRpy0 = imuData_.rpy;
    tempRpy0(2) = 0.0;
    Mat3<double> tempBodyR0 = rpyToRotMat(tempRpy0).transpose();
    aTemp = tempBodyR0 * imuData_.a;
    for (int i = 0; i < 3; i++) {
        output_.imuVec(i) = imuData_.rpy(i);
        output_.imuVec(i + 3) = imuData_.w(i);
        output_.imuVec(i + 6) = aTemp(i);
    }

    output_.pFoot = forwardKinematics(motorData_.alpha) + qrParam.pRoll;  // 身体系下

    output_.vFoot = calFootVelocity(motorData_.alpha, motorData_.torq);

    output_.vCom = calBodyMeanVelocity(output_.vFoot, output_.realContactFlag);
    output_.rtHeight = -output_.pFoot.row(2);
    output_.bodyHeight = calBodyHeight(output_.rtHeight, jpData_.contactFlag, output_.realContactFlag);

    output_.groundR.setIdentity();

    output_.flag.fallDownInfo = calFallDown(imuData_.rpy, sta);

    double muc[2], variance[2];
    muc[0] = 0.0;
    muc[1] = 1.0;
    double muzg = 0;
    double varianceZg = 0.05;

    variance[0] = 0.001;
    variance[1] = 0.05;
    double mufc = 15;
    double varianceFc = 300;
    double Pc = 0.4;

    std::pair<bool, Vec2<double>> tmpResult;
    Vec4<bool> mitContact;
    for (int i = 0; i < 4; i++) {
        if (jpData_.sPhi(i) == 0 && jpData_.phi(i) > 0 && jpData_.phi(i) < 0.4) {
            output_.realContactFlag(i) = false;
            // cout << "reset ok" << endl;
        }

        tmpResult = contactDetector(jpData_.sPhi(i),
                                    jpData_.phi(i),
                                    muc,
                                    variance,
                                    (output_.pFoot(2, i) + jpData_.desireHeight),
                                    muzg,
                                    varianceZg,
                                    -output_.fFoot(2, i),
                                    mufc,
                                    varianceFc,
                                    Pc);

        output_.pRef.col(i).fill(tmpResult.second(0));

        if (output_.realContactFlag(i) == false) {
            output_.realContactFlag(i) = tmpResult.first;
        }
    }
}

Vec4<bool> BasicEstimator::realContactCheckMiddle(Mat34<double> fFoot)
{
    Vec4<bool> flagVec;
    flagVec.fill(false);
    for (int i = 0; i < 4; i++) {
        fFootData(i).push_back(fFoot(2, i));
        if (fFootData(i).size() > 10) {
            fFootData(i).pop_front();
            if (fFootData(i).at(0) > 0) {
                if (i == 0) {
                    cout << "fFootData " << fFootData(i).at(0) << " " << fFootData(i).at(9) << endl;
                }
                if ((fFootData(i).at(9) / fFootData(i).at(0)) > 1.5) {
                    flagVec(i) = true;
                }
            }
        }
    }
    // cout << "flagVec: " << flagVec(0) << endl;
    return flagVec;
}
Vec4<bool> BasicEstimator::realContactCheck(Mat34<double> fFoot, double valueSim, double valueReal)
{
    Vec4<bool> flagVec;
    for (int i = 0; i < 4; i++) {
        if (qrParam.model.back() == QrModel::base) {
            if ((fFoot.col(i).norm() > valueSim) && (fFoot(2, i) < 0.0)) {
                flagVec(i) = true;
            } else {
                flagVec(i) = false;
            }
        } else {
            if ((fFoot.col(i).norm() > valueReal) && (fFoot(2, i) < -10.0)) {
                flagVec(i) = true;
            } else {
                flagVec(i) = false;
            }
        }
    }
    // cout << "fFoot = " << fFoot << endl;
    return flagVec;
}

Mat34<double> BasicEstimator::forwardDynamics(Mat34<double> tau, Mat34<double> q)
{
    Vec4<bool> isFR;
    Vec3<double> Q;
    Mat3<double> JacobianT;
    Mat34<double> temp_forceFoot;
    isFR << true, false, true, false;
    Q.setZero();
    JacobianT.setZero();
    temp_forceFoot.setZero();

    for (int i = 0; i < 4; i++) {
        Q = q.col(i);
        double sigma3 = cos(Q(1) + Q(2));
        double sigma2 = cos(Q(1)) * qrParam.LEG_L1 + sigma3 * qrParam.LEG_L2;
        double sigma1 = sin(Q(1)) * qrParam.LEG_L1 + sin(Q(1) + Q(2)) * qrParam.LEG_L2;
        if (isFR[i] == true) {
            JacobianT << 0.0, sin(Q(0)) * qrParam.LEG_L0 + cos(Q(0)) * sigma1, sin(Q(0)) * sigma1 - cos(Q(0)) * qrParam.LEG_L0, sigma1, sin(Q(0)) * sigma2, -cos(Q(0)) * sigma2,
                sin(Q(1) + Q(2)) * qrParam.LEG_L2, sin(Q(0)) * sigma3 * qrParam.LEG_L2, -cos(Q(0)) * sigma3 * qrParam.LEG_L2;
        } else {
            JacobianT << 0.0, sin(Q(0)) * qrParam.LEG_L0 * -1.0 + cos(Q(0)) * sigma1, sin(Q(0)) * sigma1 - cos(Q(0)) * qrParam.LEG_L0 * -1.0, sigma1, sin(Q(0)) * sigma2,
                -cos(Q(0)) * sigma2, sin(Q(1) + Q(2)) * qrParam.LEG_L2, sin(Q(0)) * sigma3 * qrParam.LEG_L2, -cos(Q(0)) * sigma3 * qrParam.LEG_L2;
        }

        //(*tau).col(i) = JacobianT * (*f).col(i);
        temp_forceFoot.col(i) = JacobianT.inverse() * tau.col(i);
    }

    return temp_forceFoot;
}

Mat34<double> BasicEstimator::forwardKinematics(Mat34<double> q)
{
    Vec4<bool> isFR;
    isFR << true, false, true, false;
    Mat34<double> p;

    for (int i = 0; i < 4; i++) {
        p(0, i) = -qrParam.LEG_L1 * cos(q(1, i)) - qrParam.LEG_L2 * cos(q(1, i) + q(2, i));
        if (!isFR(i)) {
            p(1, i) = (-qrParam.LEG_L1 * sin(q(1, i)) - qrParam.LEG_L2 * sin(q(1, i) + q(2, i))) * -sin(q(0, i)) + qrParam.LEG_L0 * cos(q(0, i));
            p(2, i) = (-qrParam.LEG_L1 * sin(q(1, i)) - qrParam.LEG_L2 * sin(q(1, i) + q(2, i))) * cos(q(0, i)) + qrParam.LEG_L0 * sin(q(0, i));
        } else {
            p(1, i) = (-qrParam.LEG_L1 * sin(q(1, i)) - qrParam.LEG_L2 * sin(q(1, i) + q(2, i))) * -sin(q(0, i)) - qrParam.LEG_L0 * cos(q(0, i));
            p(2, i) = (-qrParam.LEG_L1 * sin(q(1, i)) - qrParam.LEG_L2 * sin(q(1, i) + q(2, i))) * cos(q(0, i)) - qrParam.LEG_L0 * sin(q(0, i));
        }
    }
    return p;
}

double BasicEstimator::calBodyHeight(Vec4<double> rtH, Vec4<bool> contactFlag, Vec4<bool> realContactFlag)
{
    (void)realContactFlag;
    if ((_curState == AlgoState::slowDown) || (_curState == AlgoState::walk)) {
        int tmpContactNum = 0;
        double sumH = 0.0;
        for (int i = 0; i < 4; i++) {
            if (contactFlag(i) == true) {
                tmpContactNum++;
                sumH += rtH(i);
            }
        }

        if (tmpContactNum == 0) {
            cout << "contact err!" << endl;
            return qrParam.bodyHeight;
        } else {
            return sumH / tmpContactNum;
        }
    } else {
        // cout << "height " << rtH.sum() / 4 << endl;
        return rtH.sum() / 4;
    }
}

Mat34<double> BasicEstimator::calFootVelocity(Mat34<double> q, Mat34<double> qd)
{
    Vec4<bool> isFR;
    isFR << true, false, true, false;
    Vec3<double> Q;
    Mat3<double> Jacobian;
    Mat34<double> vfoot;
    Q.setZero();
    Jacobian.setZero();
    vfoot.setZero();

    for (int i = 0; i < 4; i++) {
        Q = q.col(i);
        double sigma3 = cos(Q(1) + Q(2));
        double sigma2 = cos(Q(1)) * qrParam.LEG_L1 + sigma3 * qrParam.LEG_L2;
        double sigma1 = sin(Q(1)) * qrParam.LEG_L1 + sin(Q(1) + Q(2)) * qrParam.LEG_L2;
        if (isFR(i) == true) {
            Jacobian << 0.0, sigma1, sin(Q(1) + Q(2)) * qrParam.LEG_L2, sin(Q(0)) * qrParam.LEG_L0 + cos(Q(0)) * sigma1, sin(Q(0)) * sigma2, sin(Q(0)) * sigma3 * qrParam.LEG_L2,
                sin(Q(0)) * sigma1 - cos(Q(0)) * qrParam.LEG_L0, -cos(Q(0)) * sigma2, -cos(Q(0)) * sigma3 * qrParam.LEG_L2;
        } else {
            Jacobian << 0.0, sigma1, sin(Q(1) + Q(2)) * qrParam.LEG_L2, sin(Q(0)) * -qrParam.LEG_L0 + cos(Q(0)) * sigma1, sin(Q(0)) * sigma2, sin(Q(0)) * sigma3 * qrParam.LEG_L2,
                sin(Q(0)) * sigma1 - cos(Q(0)) * -qrParam.LEG_L0, -cos(Q(0)) * sigma2, -cos(Q(0)) * sigma3 * qrParam.LEG_L2;
        }

        vfoot.col(i) = Jacobian * qd.col(i);
    }
    return vfoot;
}

Mat3<double> BasicEstimator::calFootR(Mat34<double> pFootCur, Mat34<double> fFootCur, Vec4<bool> contactFlagCur)
{
    // 对角线两腿和任意单腿都有问题，没想到好的办法,王兴兴论文中也指的是3或4足落地的情况做计算
    Mat3<double> R;
    Vec3<double> rpy;
    Vec4<double> fFootNorm;
    // 得到有效支撑的腿数目
    int temp_contactNum = 0;
    for (int i = 0; i < 4; i++) {
        fFootNorm(i) = fFootCur.col(i).norm();  // 求出每个足部的力的幅值填入；

        if (contactFlagCur(i)) {
            temp_contactNum++;
        }
        // cout << "contactFlagCur = " << contactFlagCur << endl;
    }
    // 根据不同的腿数目，用不同方法构造平面
    Vec3<double> r1, r2;  // 构造平面的两个矢量,r1是右边指向左边的宽度方向矢量，r2是后边指向前边，长度方向的矢量
    switch (temp_contactNum) {
        case 3:
            // cout << "temp_contactNum = " << temp_contactNum << endl;
            if (contactFlagCur(0) == 0) {
                r1 = pFootCur.col(3) - pFootCur.col(2);
                r2 = pFootCur.col(1) - pFootCur.col(3);
            } else if (contactFlagCur(1) == 0) {
                r1 = pFootCur.col(3) - pFootCur.col(2);
                r2 = pFootCur.col(0) - pFootCur.col(3);
            } else if (contactFlagCur(2) == 0) {
                r1 = pFootCur.col(1) - pFootCur.col(0);
                r2 = pFootCur.col(0) - pFootCur.col(3);
            } else {
                r1 = pFootCur.col(1) - pFootCur.col(0);
                r2 = pFootCur.col(0) - pFootCur.col(2);
            }
            break;
        case 4:
        case 0:
            // cout << "temp_contactNum = " << temp_contactNum << endl;
            MatrixXd::Index minRow, minCol;
            fFootNorm.minCoeff(&minRow, &minCol);  // 找到最小元素的行与列,力最小的那个腿弃用,Vec4是列向量存储，所以用ROW
            if (minRow == 0) {
                r1 = pFootCur.col(3) - pFootCur.col(2);
                r2 = pFootCur.col(1) - pFootCur.col(3);
            } else if (minRow == 1) {
                r1 = pFootCur.col(3) - pFootCur.col(2);
                r2 = pFootCur.col(0) - pFootCur.col(3);
            } else if (minRow == 2) {
                r1 = pFootCur.col(1) - pFootCur.col(0);
                r2 = pFootCur.col(0) - pFootCur.col(3);
            } else {
                r1 = pFootCur.col(1) - pFootCur.col(0);
                r2 = pFootCur.col(0) - pFootCur.col(2);
            }
            break;
        case 2:
            cout << "temp_contactNum = " << temp_contactNum << endl;
            // 01 02 03 12 13 23 C42=6种   先求两个点的中点，根据BodyLxLy作偏置的到第三个理想点，构造平面
            if (contactFlagCur(0) == 0 && contactFlagCur(1) == 0) {
                // 23 = true
                r1 = pFootCur.col(3) - pFootCur.col(2);
                Vec3<double> tmpVec = (pFootCur.col(3) + pFootCur.col(2)) / 2;
                tmpVec(0) += qrParam.BODY_Lx;  // 生成随动的头部下方位置矢量
                r2 = tmpVec - pFootCur.col(3);
            } else if (contactFlagCur(0) == 0 && contactFlagCur(2) == 0) {
                // 13 = true
                r2 = pFootCur.col(1) - pFootCur.col(3);
                Vec3<double> tmpVec = (pFootCur.col(1) + pFootCur.col(3)) / 2;
                tmpVec(1) -= qrParam.BODY_Ly;
                r1 = pFootCur.col(3) - tmpVec;
            } else if (contactFlagCur(0) == 0 && contactFlagCur(3) == 0) {
                // 12 = true
                r1 = pFootCur.col(1) - pFootCur.col(2);
                Vec3<double> tmpVec = (pFootCur.col(1) + pFootCur.col(2)) / 2;
                tmpVec(0) += qrParam.BODY_Lx * 0.5;
                r2 = tmpVec - pFootCur.col(2);
            } else if (contactFlagCur(1) == 0 && contactFlagCur(2) == 0) {
                // 03 = true
                r1 = pFootCur.col(0) - pFootCur.col(3);
                Vec3<double> tmpVec = (pFootCur.col(0) + pFootCur.col(3)) / 2;
                tmpVec(0) += qrParam.BODY_Lx * 0.5;
                r2 = tmpVec - pFootCur.col(3);
            } else if (contactFlagCur(1) == 0 && contactFlagCur(3) == 0) {
                // 02 = true
                r2 = pFootCur.col(0) - pFootCur.col(2);
                Vec3<double> tmpVec = (pFootCur.col(0) + pFootCur.col(2)) / 2;
                tmpVec(1) += qrParam.BODY_Ly;
                r1 = tmpVec - pFootCur.col(2);
            } else {
                // 01 = true
                r1 = pFootCur.col(1) - pFootCur.col(0);
                Vec3<double> tmpVec = (pFootCur.col(2) + pFootCur.col(3)) / 2;  // 未触底两个足部构造一个平面
                r2 = pFootCur.col(0) - tmpVec;
            }
            break;
        case 1:
            cout << "temp_contactNum = " << temp_contactNum << endl;
            // 只有四种情况
            {
                Vec3<double> p1, p2;
                if (contactFlagCur(0) == 1) {
                    p1 = pFootCur.col(0);  // 右前腿位置矢量
                    p1(1) = 0;             // 构造第一个点
                    p2 = p1;
                    p2(0) -= qrParam.BODY_Lx;  // 构造第二个点
                    r1 = p1 - pFootCur.col(0);
                    r2 = p1 - p2;
                } else if (contactFlagCur(1) == 1) {
                    p1 = pFootCur.col(1);  // 左前腿位置矢量
                    p1(1) = 0;             // 构造第一个点
                    p2 = p1;
                    p2(0) -= qrParam.BODY_Lx;  // 构造第二个点
                    r1 = pFootCur.col(1) - p1;
                    r2 = p1 - p2;
                } else if (contactFlagCur(2) == 1) {
                    p1 = pFootCur.col(2);  // 右后腿位置矢量
                    p1(1) = 0;             // 构造第一个点
                    p2 = p1;
                    p2(0) += qrParam.BODY_Lx;  // 构造第二个点
                    r1 = p1 - pFootCur.col(2);
                    r2 = p2 - p1;
                } else {
                    p1 = pFootCur.col(3);  // 左后腿位置矢量
                    p1(1) = 0;             // 构造第一个点
                    p2 = p1;
                    p2(0) += qrParam.BODY_Lx;  // 构造第二个点
                    r1 = pFootCur.col(3) - p1;
                    r2 = p2 - p1;
                }
                break;
            }
        default:
            cout << "real contact num err!" << endl;
            break;
    }
    // 处理好了r1,r2的取值，进一步计算
    Vec3<double> z = r2.cross(r1);
    Vec3<double> Z = z / z.norm();
    Vec3<double> Y = r1 / r1.norm();
    Vec3<double> X = Y.cross(Z);  // 两个正交的单位矢量叉乘一定是单位矢量

    R << X.transpose(), Y.transpose(), Z.transpose();
    rpy = rotationMatrixToRPY(R);
    rpy(2) = 0.0;          // 去掉yaw角，IMU返回也是不带yaw的
    R = rpyToRotMat(rpy);  // 足部平面姿态相对于身体的姿态矩阵
    return R;
}

Vec3<double> BasicEstimator::calGroundR2Body(Mat34<double> pFootCur, Vec4<bool> contactFlagCur, Vec3<double> rpyOld)
{
    Mat3<double> R;
    Vec3<double> rpy;
    int temp_contactNum = 0;
    for (int i = 0; i < 4; i++) {
        if (contactFlagCur(i)) {
            temp_contactNum++;
        }
    }

    // 以下计算基于三腿支撑
    if (temp_contactNum == 3) {
        Vec3<double> r1, r2;
        if (contactFlagCur(0) == 0) {
            r1 = pFootCur.col(3) - pFootCur.col(2);
            r2 = pFootCur.col(1) - pFootCur.col(3);
        } else if (contactFlagCur(1) == 0) {
            r1 = pFootCur.col(3) - pFootCur.col(2);
            r2 = pFootCur.col(0) - pFootCur.col(3);
        } else if (contactFlagCur(2) == 0) {
            r1 = pFootCur.col(1) - pFootCur.col(0);
            r2 = pFootCur.col(0) - pFootCur.col(3);
        } else {
            r1 = pFootCur.col(1) - pFootCur.col(0);
            r2 = pFootCur.col(0) - pFootCur.col(2);
        }

        Vec3<double> z = r2.cross(r1);
        Vec3<double> Z = z / z.norm();
        Vec3<double> Y = r1 / r1.norm();
        Vec3<double> X = Y.cross(Z);  // 两个正交的单位矢量叉乘一定是单位矢量

        R << X.transpose(), Y.transpose(), Z.transpose();
        rpy = rotationMatrixToRPY(R);
        rpy(2) = 0.0;  // 去掉yaw角，IMU返回也是不带yaw的
        // R = rpyToRotMat(rpy);  // 足部平面姿态相对于身体的姿态矩阵
        return rpy;
    } else if (temp_contactNum == 4) {
        Mat34<double> rpy4;
        for (int i = 0; i < 4; i++) {
            Vec3<double> r1, r2;
            if (i == 0) {
                r1 = pFootCur.col(3) - pFootCur.col(2);
                r2 = pFootCur.col(1) - pFootCur.col(3);
            } else if (i == 1) {
                r1 = pFootCur.col(3) - pFootCur.col(2);
                r2 = pFootCur.col(0) - pFootCur.col(3);
            } else if (i == 2) {
                r1 = pFootCur.col(1) - pFootCur.col(0);
                r2 = pFootCur.col(0) - pFootCur.col(3);
            } else {
                r1 = pFootCur.col(1) - pFootCur.col(0);
                r2 = pFootCur.col(0) - pFootCur.col(2);
            }

            Vec3<double> z = r2.cross(r1);
            Vec3<double> Z = z / z.norm();
            Vec3<double> Y = r1 / r1.norm();
            Vec3<double> X = Y.cross(Z);  // 两个正交的单位矢量叉乘一定是单位矢量

            R << X.transpose(), Y.transpose(), Z.transpose();
            rpy4.col(i) = rotationMatrixToRPY(R);
        }

        rpy(0) = rpy4.row(0).sum() / 4.0;
        rpy(1) = rpy4.row(1).sum() / 4.0;
        rpy(2) = 0.0;
        return rpy;
        // R = rpyToRotMat(rpy);
    } else {
        rpy = rpyOld;
        return rpy;
    }
    // return R;
    // return rotationMatrixToRPY(R);
}

Mat3<double> BasicEstimator::calGroundR(Mat34<double> pFootCur, Vec4<bool> contactFlagCur, Mat3<double> imuR)
{
    Mat3<double> R;
    Vec3<double> rpy;

    int temp_contactNum = 0;
    for (int i = 0; i < 4; i++) {
        if (contactFlagCur(i)) {
            temp_contactNum++;
        }
    }
    /*
    //四腿支撑时，处理成三腿支撑
    Vec4<double> legLenthi;
    if (temp_contactNum == 4) {
        for (int i = 0; i < 4; i++) {
            legLenthi(i) = pFootCur.col(i).norm();
        }
        int MAXCOL, MAXROW;
        legLenthi.maxCoeff(&MAXROW, &MAXCOL);
        contactFlagCur(MAXROW) = false;
        temp_contactNum = 3;
        cout << "contactFlagCur = " << contactFlagCur << endl;
    }
    */

    // cout << "temp_contactNum = " << temp_contactNum << endl;
    /*
    //双腿支撑时，处理成三腿,导致双腿支撑时，平衡能力变差
    if (temp_contactNum == 2) {
        if (contactFlagCur(0)) {
            // pFootCur.col(2) = pFootCur.col(3);
            pFootCur(0, 2) = pFootCur(0, 3);
            pFootCur(1, 2) = -pFootCur(1, 3);
            pFootCur(2, 2) = pFootCur(2, 3);
            contactFlagCur(2) = true;
        } else {
            pFootCur(0, 3) = pFootCur(0, 2);
            pFootCur(1, 3) = -pFootCur(1, 2);
            pFootCur(2, 3) = pFootCur(2, 2);
            contactFlagCur(3) = true;
        }
        temp_contactNum = 3;
    }
    */
    // 以下计算基于三腿支撑
    if (temp_contactNum == 3) {
        Vec3<double> r1, r2;
        if (contactFlagCur(0) == 0) {
            r1 = pFootCur.col(3) - pFootCur.col(2);
            r2 = pFootCur.col(1) - pFootCur.col(3);
        } else if (contactFlagCur(1) == 0) {
            r1 = pFootCur.col(3) - pFootCur.col(2);
            r2 = pFootCur.col(0) - pFootCur.col(3);
        } else if (contactFlagCur(2) == 0) {
            r1 = pFootCur.col(1) - pFootCur.col(0);
            r2 = pFootCur.col(0) - pFootCur.col(3);
        } else {
            r1 = pFootCur.col(1) - pFootCur.col(0);
            r2 = pFootCur.col(0) - pFootCur.col(2);
        }

        Vec3<double> z = r2.cross(r1);
        Vec3<double> Z = z / z.norm();
        Vec3<double> Y = r1 / r1.norm();
        Vec3<double> X = Y.cross(Z);  // 两个正交的单位矢量叉乘一定是单位矢量

        R << X.transpose(), Y.transpose(), Z.transpose();
        rpy = rotationMatrixToRPY(R);
        rpy(2) = 0.0;          // 去掉yaw角，IMU返回也是不带yaw的
        R = rpyToRotMat(rpy);  // 足部平面姿态相对于身体的姿态矩阵
    } else if (temp_contactNum == 4) {
        Mat34<double> rpy4;
        for (int i = 0; i < 4; i++) {
            Vec3<double> r1, r2;
            if (i == 0) {
                r1 = pFootCur.col(3) - pFootCur.col(2);
                r2 = pFootCur.col(1) - pFootCur.col(3);
            } else if (i == 1) {
                r1 = pFootCur.col(3) - pFootCur.col(2);
                r2 = pFootCur.col(0) - pFootCur.col(3);
            } else if (i == 2) {
                r1 = pFootCur.col(1) - pFootCur.col(0);
                r2 = pFootCur.col(0) - pFootCur.col(3);
            } else {
                r1 = pFootCur.col(1) - pFootCur.col(0);
                r2 = pFootCur.col(0) - pFootCur.col(2);
            }

            Vec3<double> z = r2.cross(r1);
            Vec3<double> Z = z / z.norm();
            Vec3<double> Y = r1 / r1.norm();
            Vec3<double> X = Y.cross(Z);  // 两个正交的单位矢量叉乘一定是单位矢量

            R << X.transpose(), Y.transpose(), Z.transpose();
            rpy4.col(i) = rotationMatrixToRPY(R);
        }

        rpy(0) = rpy4.row(0).sum() / 4.0;
        rpy(1) = rpy4.row(1).sum() / 4.0;
        rpy(2) = 0.0;
        R = rpyToRotMat(rpy);
    } else {
        R = imuR.transpose();  // 双足支撑时为imu的姿态角负号,转置
    }
    return R;
}

Vec3<double> BasicEstimator::calBodyMeanVelocity(Mat34<double> vFootMat, Vec4<bool> contactFlagVec)
{
    int tmpContactNum = 0;
    Vec3<double> vFootSum;
    vFootSum.setZero();
    Vec3<double> tmpV;
    for (int i = 0; i < 4; i++) {
        if (contactFlagVec(i)) {
            tmpContactNum++;
            vFootSum += vFootMat.col(i);
        }
    }

    for (int i = 0; i < 99; i++) {
        vCom100.col(i) = vCom100.col(i + 1);
    }
    // 注意，如果realcontactFlag全是false，会导致0 / 0 这里必须要进行处理
    if (tmpContactNum != 0) {
        vCom100.col(99) = -vFootSum / tmpContactNum;
    } else {
        vCom100.col(99) = Vec3<double>(0.0, 0.0, 0.0);  // 塞0
    }

    tmpV(0) = vCom100.row(0).sum() / 100;
    tmpV(1) = vCom100.row(1).sum() / 100;
    tmpV(2) = vCom100.row(2).sum() / 100;

    return tmpV;
}

int BasicEstimator::calFallDown(Vec3<double> rpyCur, AlgoState curState)
{
    (void)rpyCur;
    // if (curState == AlgoState::walk) {
    //     if ((fabs(rpyCur(0)) >= 1.5) || (rpyCur(1) >= 1.57) || (rpyCur(1) <= -2.1)) {
    //         cout << "rpyCur = " << rpyCur << endl;
    //         if (rpyCur(0) >= 0) {
    //             return 1;
    //         } else {
    //             return -1;
    //         }

    //     } else {
    //         return 0;
    //     }

    // } else {
    //     return 0;
    // }
    if (curState == AlgoState::walk || curState == AlgoState::upright || curState == AlgoState::handStand || curState == AlgoState::stand) {
        Vec4<double> q = output_.quat;
        // Vec4<double> q{0.0, 0.64, 0.0, 0.76};
        Vec3<double> v{0, 0, -1};
        Vec3<double> tempVec = mathFun.QuatRotateInverse(q, v);
        // cout << "q: " << q << endl;
        // cout << "tempVec" << tempVec << endl;
        if (tempVec(2) >= 0.5) {
            cout << "current Fall status: " << tempVec << endl;
            if (tempVec(1) >= 0)
                return 1;
            else if (tempVec(1) < 0)
                return -1;
            // return 1;
        } else {
            return 0;
        }
    } else {
        return 0;
    }

    // cout << mathFun.QuatRotateInverse(q, v) << endl;
    return 0;
}
// 添加基于概率的触地检测模型
std::pair<bool, Vec2<double>> BasicEstimator::contactDetector(double sPhi, double phi, double muc[2], double variance[2], double pz, double muzg, double varianceZg, double fz,
                                                              double mufc, double varianceFc, double Pc)
{
    std::pair<bool, Vec2<double>> result;
    double sigma_idx_0;
    double sigma_idx_1;
    double u;
    sigma_idx_0 = std::sqrt(variance[0]);
    sigma_idx_1 = std::sqrt(variance[1]);
    if (sPhi == 0.0) {
        u = 0.5 * (2.0 + (b_erf((muc[0] - phi) / (1.4142135623730951 * sigma_idx_0)) + b_erf((phi - muc[1]) / (1.4142135623730951 * sigma_idx_1))));
    } else {
        u = 0.5 * (b_erf((phi - muc[0]) / (1.4142135623730951 * sigma_idx_0)) + b_erf((muc[1] - phi) / (1.4142135623730951 * sigma_idx_1)));
    }

    sigma_idx_0 = 0.5 * (1.0 + b_erf((muzg - pz) / (std::sqrt(varianceZg) * 1.4142135623730951)));
    sigma_idx_1 = 0.5 * (1.0 + b_erf((fz - mufc) / (std::sqrt(varianceFc) * 1.4142135623730951)));

    // z这里只计算一个腿，所以矩阵只有一个元素
    // kalman filter
    sigma_idx_0 = u + ((variance[0] * (variance[0] + varianceZg) + variance[0] * variance[0]) * (sigma_idx_0 - u) +
                       (variance[0] * variance[0] + variance[0] * (variance[0] + varianceFc)) * (sigma_idx_1 - u));

    //  p = (1 - K * H) * p;  %由于A=0，这个值没有用
    // 判断是否超过阈值，超过则为1，否则为0
    result.second(0) = u;
    result.second(1) = sigma_idx_0;
    result.first = (sigma_idx_0 > Pc);
    return result;
}

double BasicEstimator::b_erf(double x)
{
    double y;
    double absx;
    double S;
    double s;
    double R;
    int eint;
    absx = std::fabs(x);
    if (mathFun.rtIsNaN(x)) {
        y = x;
    } else if (mathFun.rtIsInf(x)) {
        if (x < 0.0) {
            y = -1.0;
        } else {
            y = 1.0;
        }
    } else if (absx < 0.84375) {
        if (absx < 3.7252902984619141E-9) {
            if (absx < 2.8480945388892178E-306) {
                y = 0.125 * (8.0 * x + 1.0270333367641007 * x);
            } else {
                y = x + 0.12837916709551259 * x;
            }
        } else {
            s = x * x;
            y = x + x * ((s * (s * (s * (s * -2.3763016656650163E-5 + -0.0057702702964894416) + -0.02848174957559851) + -0.3250421072470015) + 0.12837916709551256) /
                         (s * (s * (s * (s * (s * -3.9602282787753681E-6 + 0.00013249473800432164) + 0.0050813062818757656) + 0.0650222499887673) + 0.39791722395915535) + 1.0));
        }
    } else if (absx < 1.25) {
        S = (absx - 1.0) * ((absx - 1.0) * ((absx - 1.0) * ((absx - 1.0) * ((absx - 1.0) * ((absx - 1.0) * -0.0021663755948687908 + 0.035478304325618236) + -0.11089469428239668) +
                                                            0.31834661990116175) +
                                            -0.37220787603570132) +
                            0.41485611868374833) +
            -0.0023621185607526594;
        s = (absx - 1.0) * ((absx - 1.0) * ((absx - 1.0) * ((absx - 1.0) * ((absx - 1.0) * ((absx - 1.0) * 0.011984499846799107 + 0.013637083912029051) + 0.12617121980876164) +
                                                            0.071828654414196266) +
                                            0.540397917702171) +
                            0.10642088040084423) +
            1.0;
        if (x >= 0.0) {
            y = S / s + 0.84506291151046753;
        } else {
            y = -0.84506291151046753 - S / s;
        }
    } else if (absx > 6.0) {
        if (x < 0.0) {
            y = -1.0;
        } else {
            y = 1.0;
        }
    } else {
        s = 1.0 / (absx * absx);
        if (absx < 2.8571434020996094) {
            R = s * (s * (s * (s * (s * (s * (s * -9.8143293441691455 + -81.2874355063066) + -184.60509290671104) + -162.39666946257347) + -62.375332450326006) +
                          -10.558626225323291) +
                     -0.69385857270718176) +
                -0.0098649440348471482;
            S = s * (s * (s * (s * (s * (s * (s * (s * -0.0604244152148581 + 6.5702497703192817) + 108.63500554177944) + 429.00814002756783) + 645.38727173326788) +
                               434.56587747522923) +
                          137.65775414351904) +
                     19.651271667439257) +
                1.0;
        } else {
            R = s * (s * (s * (s * (s * (s * -483.5191916086514 + -1025.0951316110772) + -637.56644336838963) + -160.63638485582192) + -17.757954917754752) + -0.799283237680523) +
                -0.0098649429247001;
            S = s * (s * (s * (s * (s * (s * (s * -22.440952446585818 + 474.52854120695537) + 2553.0504064331644) + 3199.8582195085955) + 1536.729586084437) + 325.79251299657392) +
                     30.338060743482458) +
                1.0;
        }

        if (!mathFun.rtIsNaN(absx)) {
            s = frexp(absx, &eint);
        } else {
            s = absx;
            eint = 0;
        }

        s = std::floor(s * 2.097152E+6) / 2.097152E+6 * rt_powd_snf(2.0, static_cast<double>(eint));
        y = std::exp(-s * s - 0.5625) * std::exp((s - absx) * (s + absx) + R / S) / absx;
        if (x < 0.0) {
            y--;
        } else {
            y = 1.0 - y;
        }
    }

    return y;
}

double BasicEstimator::rt_powd_snf(double u0, double u1)
{
    double y;
    double d;
    double d1;
    if (mathFun.rtIsNaN(u0) || mathFun.rtIsNaN(u1)) {
        y = mathFun.rtNaN;
    } else {
        d = std::fabs(u0);
        d1 = std::fabs(u1);
        if (mathFun.rtIsInf(u1)) {
            if (d == 1.0) {
                y = 1.0;
            } else if (d > 1.0) {
                if (u1 > 0.0) {
                    y = mathFun.rtInf;
                } else {
                    y = 0.0;
                }
            } else if (u1 > 0.0) {
                y = 0.0;
            } else {
                y = mathFun.rtInf;
            }
        } else if (d1 == 0.0) {
            y = 1.0;
        } else if (d1 == 1.0) {
            if (u1 > 0.0) {
                y = u0;
            } else {
                y = 1.0 / u0;
            }
        } else if (u1 == 2.0) {
            y = u0 * u0;
        } else if ((u1 == 0.5) && (u0 >= 0.0)) {
            y = std::sqrt(u0);
        } else if ((u0 < 0.0) && (u1 > std::floor(u1))) {
            y = mathFun.rtNaN;
        } else {
            y = pow(u0, u1);
        }
    }

    return y;
}

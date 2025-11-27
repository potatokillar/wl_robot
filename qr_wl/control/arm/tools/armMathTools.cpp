
#include "armMathTools.hpp"

#include <iostream>

#include "baseline.hpp"

using namespace ::std;
/**
 * @description:atan2的结果(-pi~pi)改变值域
 * @param {double} value atan2的值域(-pi~pi)
 * @return {double} Roll值域(0~2Pi)
 */
double ArmMathFun::atan2toRoll(double value)
{
    if (value < 0) {
        return (value + 2 * M_PI);
    }
    return value;
}
/**
 * @description:绕z轴的旋转矩阵,参考百度百科表达式
 * @param {Vec3<double>} pIn需要旋转的矢量
 * @param {double} radZ需要绕z轴旋转的弧度
 * @return {Vec3<double>}旋转后的矢量
 */
Vec3<double> ArmMathFun::Rz(const Vec3<double>& pIn, double radZ)
{
    Mat3<double> Rz;
    Rz.setIdentity();
    Rz << cos(radZ), -sin(radZ), 0.0, sin(radZ), cos(radZ), 0.0, 0.0, 0.0, 1.0;
    return (Rz * pIn);
}
/**
 * @description:绕y轴的旋转矩阵,参考百度百科表达式
 * @param {Vec3<double>} pIn需要旋转的矢量
 * @param {double} radY需要绕y轴旋转的弧度
 * @return {Vec3<double>}旋转后的矢量
 */
Vec3<double> ArmMathFun::Ry(const Vec3<double>& pIn, double radY)
{
    Mat3<double> Ry;
    Ry.setIdentity();
    Ry << cos(radY), 0.0, sin(radY), 0.0, 1.0, 0.0, -sin(radY), 0.0, cos(radY);
    return (Ry * pIn);
}
/**
 * @description:绕x轴的旋转矩阵，参考百度百科表达式
 * @param {Vec3<double>} pIn需要旋转的矢量
 * @param {double} radX需要绕x轴旋转的弧度
 * @return {Vec3<double>}旋转后的矢量
 */
Vec3<double> ArmMathFun::Rx(const Vec3<double>& pIn, double radX)
{
    Mat3<double> Rx;
    Rx.setIdentity();
    Rx << 1.0, 0.0, 0.0, 0.0, cos(radX), -sin(radX), 0.0, sin(radX), cos(radX);
    return (Rx * pIn);
}
/**
 * @description:四元数转欧拉角，来源于MIT
 * @param {Vec4<double>} q四元数
 * @return {Vec3<double>}ZYX欧拉角(yaw,pitch,roll)
 */
Vec3<double> ArmMathFun::quaternion2Euler(const Vec4<double>& q)
{
    // 注意：绕固定坐标系旋转，依次绕 X、Y、Z转过roll，pitch，yaw 的旋转矩阵 ==
    // 绕动坐标系连续旋转，依次绕Z、Y、X轴转roll，pitch，yaw的旋转矩阵，也称ZYX欧拉角，参看机器人书
    Vec3<double> euler;
    euler.setZero();
    double sinr_cosp = 2.0 * (q(0) * q(1) + q(2) * q(3));
    double cosr_cosp = 1 - 2 * (q(1) * q(1) + q(2) * q(2));
    euler(0) = atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (q(0) * q(2) - q(3) * q(1));
    if (fabs(sinp) >= 1)
        euler(1) = copysign(M_PI / 2, sinp);  // use 90 degrees if out of range
    else
        euler(1) = asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q(0) * q(3) + q(1) * q(2));
    double cosy_cosp = 1 - 2 * (q(2) * q(2) + q(3) * q(3));
    euler(2) = atan2(siny_cosp, cosy_cosp);

    return euler;
}

Vec3<double> ArmMathFun::BezierCurve3(const Vec3<double>& p0, const Vec3<double>& p1, const Vec3<double>& pA, double s, double maxS)
{
    Vec3<double> pi;
    pi.setZero();
    s = (1.0 / maxS) * s;
    pi = p0 * pow((1 - s), 2) + 2 * pA * s * (1 - s) + p1 * pow(s, 2);
    return pi;
}

/**
 * @description:根据三阶贝塞尔曲线生成轨迹点
 * @param {Vec3<double> p0, Vec3<double> p1, Vec3<double> pA, Vec3<double>} pB 起点，终点，中间点A，中间点B
 * @param {double} s当前离散时刻
 * @param {double} maxS总离散时间
 * @return {Vec3<double>}贝塞尔曲线上的s时刻的坐标
 */
Vec3<double> ArmMathFun::BezierCurve4(const Vec3<double>& p0, const Vec3<double>& p1, const Vec3<double>& pA, const Vec3<double>& pB, double s, double maxS)
{
    Vec3<double> pi;
    pi.setZero();
    s = (1.0 / maxS) * s;
    pi = p0 * pow((1 - s), 3) + 3 * pA * s * pow((1 - s), 2) + 3 * pB * pow(s, 2) * (1 - s) + p1 * pow(s, 3);
    return pi;
}
/**
 * @description:根据四阶贝塞尔曲线生成轨迹点
 * @param {Vec3<double> p0, Vec3<double> p1, Vec3<double> pA, Vec3<double> pB, Vec3<double>} 起点，终点，中间点A，中间点B，中间点C
 * @param {double} s当前离散时刻
 * @param {double} maxS总离散时间
 * @return {Vec3<double>}贝塞尔曲线上的s时刻的坐标
 */
Vec3<double> ArmMathFun::BezierCurve5(const Vec3<double>& p0, const Vec3<double>& p1, const Vec3<double>& pA, const Vec3<double>& pB, const Vec3<double>& pC, double s, double maxS)
{
    Vec3<double> pi;
    pi.setZero();
    s = (1.0 / maxS) * s;
    pi = p0 * pow((1 - s), 4) + 4 * pA * s * pow((1 - s), 3) + 6 * pB * pow(s, 2) * pow((1 - s), 2) + 4 * pC * (1 - s) * pow(s, 3) + p1 * pow(s, 4);
    // cout << "s = " << s << endl;
    return pi;
}
Vec3<double> ArmMathFun::BezierCurve7(const Vec3<double>& p0, const Vec3<double>& p1, const Vec3<double>& pA, const Vec3<double>& pB, const Vec3<double>& pC,
                                      const Vec3<double>& pD, const Vec3<double>& pE, double s, double maxS)
{
    Vec3<double> pi;
    pi.setZero();
    s = (1.0 / maxS) * s;
    pi = p0 * pow((1 - s), 6) + 6 * pA * s * pow((1 - s), 5) + 15 * pB * pow(s, 2) * pow((1 - s), 4) + 20 * pC * pow((1 - s), 3) * pow(s, 3) + 15 * pD * pow(1 - s, 2) * pow(s, 4) +
         6 * pE * (1 - s) * pow(s, 5) + p1 * pow(s, 6);
    // cout << "s = " << s << endl;
    return pi;
}

Vec3<double> ArmMathFun::CycloidCurve(const Vec3<double>& p0, const Vec3<double>& p1, double h, double t, double T)
{
    // p取值范围为0-1
    double p = t / T;
    Vec3<double> pt;
    double phi = 2 * M_PI * p - sin(2 * M_PI * p);
    double theta = 1 - cos(2 * M_PI * p);
    pt(0) = p0(0) + (p1(0) - p0(0)) * phi / (2 * M_PI);
    pt(1) = p0(1) + (p1(1) - p0(1)) * phi / (2 * M_PI);
    pt(2) = p0(2) + (h / 2) * theta;  // 宇树的书124页表达式，这样会导致腿越走越长或越走越短,20230324做了修改起始点和圆半径，轨迹不佳，暂时弃用
    return pt;
}
/**
 * @description:根据一阶贝塞尔曲线生成轨迹点
 * @param {Vec3<double> p0, Vec3<double>} p1起点，终点
 * @param {double} s当前离散时刻
 * @param {double} maxS总离散时间
 * @return {Vec3<double>}贝塞尔曲线上的s时刻的坐标
 */
Vec3<double> ArmMathFun::BezierCurve2(const Vec3<double>& p0, const Vec3<double>& p1, double s, double maxS)
{
    Vec3<double> pi;
    pi.setZero();
    s = (1.0 / maxS) * s;
    pi = p0 * (1 - s) + p1 * s;
    return pi;
}
/**
 * @description:角度转弧度
 * @param {double} deg角度
 * @return {double} 弧度
 */
double ArmMathFun::deg2rad(double deg) { return (M_PI * deg / 180); }
/**
 * @description: 弧度转角度
 * @param {double} rad弧度
 * @return {double}角度
 */
double ArmMathFun::rad2deg(double rad) { return (180 * rad / M_PI); }
/**
 * @description: 改变atan2值域
 * @param {double} atan2Value 值域(-pi~pi)
 * @return {double} 值域(0~2pi)
 */
double ArmMathFun::changeAtan2Range(double atan2Value)
{
    if (atan2Value >= 0) {
        return atan2Value;
    }
    return (2 * M_PI + atan2Value);
}
/**
 * @description: 五次多项式插值
 * @param {double} T离散插值总时间
 * @param {double} t离散时刻
 * @return {double}值
 */
double ArmMathFun::Poly5(double T, double t)
{
    double s = 0.0;
    s = (10 / pow(T, 3)) * pow(t, 3) - (15 / pow(T, 4)) * pow(t, 4) + (6 / pow(T, 5)) * pow(t, 5);
    return s;
}
/**
 * @description: 符号函数
 * @param {double} value 值属于R
 * @return {double} 符号
 */
double ArmMathFun::Sign(double value)
{
    if (value >= 0) {
        return 1.0;
    }
    return -1.0;
}
/**
 * @description: 检查值的范围，超过截止
 * @param {double} value被检查值
 * @param {double} rangeA最小阈值
 * @param {double} rangeB最大阈值
 * @param {bool} isPrint是否打印码
 * @param {int} ErrIndex打印错误码
 * @return {double}检查后的值
 */
double ArmMathFun::setValueRange(double value, double rangeA, double rangeB, bool isPrint, int ErrIndex)
{
    double result = 0.0;
    if (rangeB >= rangeA) {
        if (value >= rangeB) {
            result = rangeB;
            if (isPrint) {
                cout << "out of rangB: " << rangeB << " ErrIndex:" << ErrIndex << endl;
            }
        } else if (value <= rangeA) {
            result = rangeA;
            if (isPrint) {
                cout << "out of rangA: " << rangeA << " ErrIndex:" << ErrIndex << endl;
            }
        } else {
            result = value;
        }
    } else {
        if (isPrint) {
            cout << "rangeA > rangeB. ErrIndex :" << ErrIndex << endl;
        }
    }
    return result;
}

double ArmMathFun::SetValueRange(double value, double min, double max, const std::string& str, bool isPrint)
{
    double result = 0.0;
    if (max >= min) {
        if (value > max) {
            result = max;
            if (setValueRangePrint_.count(str) == 0) {
                setValueRangePrint_.insert(str);
                if (isPrint) {
                    LOG_WARN("{} > MAX! max:{}, value:{}", str, max, value);
                }
            }
        } else if (value < min) {
            result = min;
            if (setValueRangePrint_.count(str) == 0) {
                setValueRangePrint_.insert(str);
                if (isPrint) {
                    LOG_WARN("{} < MIN! min:{}, value:{}", str, min, value);
                }
            }
        } else {
            result = value;
            setValueRangePrint_.erase(str);
        }
    } else {
        if (isPrint) {
            LOG_ERROR("MIN < MAX! min:{}, max:{}", min, max);
        }
    }
    return result;
}

Mat3<double> ArmMathFun::rotVec2rotMat(Vec3<double> rVec)
{
    double scale;
    double absxk;
    double t;
    double theta;
    int i;
    double u[3];
    double a[9];
    double u_tmp;
    double alpha;
    signed char b_a[9];
    double rotationVector[9];
    scale = 1.29246971E-26F;
    absxk = std::fabs(rVec(0));
    if (absxk > 1.29246971E-26F) {
        theta = 1.0F;
        scale = absxk;
    } else {
        t = absxk / 1.29246971E-26F;
        theta = t * t;
    }

    absxk = std::fabs(rVec(1));
    if (absxk > scale) {
        t = scale / absxk;
        theta = theta * t * t + 1.0F;
        scale = absxk;
    } else {
        t = absxk / scale;
        theta += t * t;
    }

    absxk = std::fabs(rVec(2));
    if (absxk > scale) {
        t = scale / absxk;
        theta = theta * t * t + 1.0F;
        scale = absxk;
    } else {
        t = absxk / scale;
        theta += t * t;
    }

    theta = scale * std::sqrt(theta);
    if (theta < 1.0E-6) {
        for (i = 0; i < 9; i++) {
            a[i] = 0.0F;
        }

        a[0] = 1.0F;
        a[4] = 1.0F;
        a[8] = 1.0F;
    } else {
        absxk = rVec(0) / theta;
        u[0] = absxk;
        t = rVec(1) / theta;
        u[1] = t;
        u_tmp = rVec(2) / theta;
        u[2] = u_tmp;
        alpha = std::cos(theta);
        scale = std::sin(theta);
        for (i = 0; i < 9; i++) {
            b_a[i] = 0;
        }

        a[0] = scale * 0.0F;
        a[3] = scale * -u_tmp;
        a[6] = scale * t;
        a[1] = scale * u_tmp;
        a[4] = scale * 0.0F;
        a[7] = scale * -absxk;
        a[2] = scale * -t;
        a[5] = scale * absxk;
        a[8] = scale * 0.0F;
        b_a[0] = 1;
        b_a[4] = 1;
        b_a[8] = 1;
        for (i = 0; i < 3; i++) {
            rotationVector[3 * i] = absxk * u[i];
            rotationVector[3 * i + 1] = t * u[i];
            rotationVector[3 * i + 2] = u_tmp * u[i];
        }

        for (i = 0; i < 9; i++) {
            a[i] = (static_cast<double>(b_a[i]) * alpha + a[i]) + (1.0F - alpha) * rotationVector[i];
        }
    }

    Mat3<double> tempR;
    for (i = 0; i < 3; i++) {
        tempR(0, i) = a[i];
        tempR(1, i) = a[i + 3];
        tempR(2, i) = a[i + 6];
    }
    return tempR;
}

void ArmMathFun::do_slerp(const double q1[4], double q2[4], double t, double q_out[4])
{
    double C;
    double qtemp_idx_0;
    double q1_inv_idx_2;
    double qtemp_idx_1;
    double qtemp_idx_2;
    double qNormalize_idx_1;
    double qNormalize_idx_2;
    double qNormalize_idx_3;
    double absxk_tmp;
    double vTheta;
    C = ((q1[0] * q2[0] + q1[1] * q2[1]) + q1[2] * q2[2]) + q1[3] * q2[3];
    if (1.0 - C <= 1.0E-9) {
        /*  avoiding divisions by number close to 0 */
        q1_inv_idx_2 = q1[0] * (1.0 - t) + q2[0] * t;
        q_out[0] = q1_inv_idx_2;
        qtemp_idx_0 = q1_inv_idx_2 * q1_inv_idx_2;
        q1_inv_idx_2 = q1[1] * (1.0 - t) + q2[1] * t;
        q_out[1] = q1_inv_idx_2;
        qtemp_idx_1 = q1_inv_idx_2 * q1_inv_idx_2;
        q1_inv_idx_2 = q1[2] * (1.0 - t) + q2[2] * t;
        q_out[2] = q1_inv_idx_2;
        qtemp_idx_2 = q1_inv_idx_2 * q1_inv_idx_2;
        q1_inv_idx_2 = q1[3] * (1.0 - t) + q2[3] * t;
        C = sqrt(((qtemp_idx_0 + qtemp_idx_1) + qtemp_idx_2) + q1_inv_idx_2 * q1_inv_idx_2);
        q_out[0] /= C;
        q_out[1] /= C;
        q_out[2] /= C;
        q_out[3] = q1_inv_idx_2 / C;
    } else {
        if (C + 1.0 <= 1.0E-9) {
            qtemp_idx_0 = q2[3];
            qtemp_idx_1 = -q2[2];
            qtemp_idx_2 = q2[1];
            C = -q2[0];

            /*  rotating one of the unit quaternions by 90 degrees -> q2 */
            q2[0] = qtemp_idx_0;
            q2[1] = qtemp_idx_1;
            q2[2] = qtemp_idx_2;
            q2[3] = C;
        }

        /*  �岹 */
        C = ((q1[0] * q1[0] + q1[1] * q1[1]) + q1[2] * q1[2]) + q1[3] * q1[3];
        qtemp_idx_2 = q1[0] / C;
        qtemp_idx_1 = -q1[1] / C;
        q1_inv_idx_2 = -q1[2] / C;
        C = -q1[3] / C;
        qtemp_idx_0 = ((qtemp_idx_2 * q2[0] - qtemp_idx_1 * q2[1]) - q1_inv_idx_2 * q2[2]) - C * q2[3];
        qNormalize_idx_1 = (qtemp_idx_2 * q2[1] + q2[0] * qtemp_idx_1) + (q1_inv_idx_2 * q2[3] - C * q2[2]);
        qNormalize_idx_2 = (qtemp_idx_2 * q2[2] + q2[0] * q1_inv_idx_2) + (C * q2[1] - qtemp_idx_1 * q2[3]);
        qNormalize_idx_3 = (qtemp_idx_2 * q2[3] + q2[0] * C) + (qtemp_idx_1 * q2[2] - q1_inv_idx_2 * q2[1]);
        C = sqrt(((qtemp_idx_0 * qtemp_idx_0 + qNormalize_idx_1 * qNormalize_idx_1) + qNormalize_idx_2 * qNormalize_idx_2) + qNormalize_idx_3 * qNormalize_idx_3);
        qtemp_idx_0 /= C;
        qNormalize_idx_1 /= C;
        qNormalize_idx_2 /= C;
        qNormalize_idx_3 /= C;
        qtemp_idx_1 = acos(qtemp_idx_0);
        q1_inv_idx_2 = sin(qtemp_idx_1);
        if (q1_inv_idx_2 != 0.0) {
            qNormalize_idx_1 /= q1_inv_idx_2;
            qNormalize_idx_2 /= q1_inv_idx_2;
            qNormalize_idx_3 /= q1_inv_idx_2;
        }

        q1_inv_idx_2 = 3.3121686421112381E-170;
        absxk_tmp = qtemp_idx_1 * qNormalize_idx_1 * t;
        C = fabs(absxk_tmp);
        if (C > 3.3121686421112381E-170) {
            vTheta = 1.0;
            q1_inv_idx_2 = C;
        } else {
            qtemp_idx_2 = C / 3.3121686421112381E-170;
            vTheta = qtemp_idx_2 * qtemp_idx_2;
        }

        qNormalize_idx_2 = qtemp_idx_1 * qNormalize_idx_2 * t;
        C = fabs(qNormalize_idx_2);
        if (C > q1_inv_idx_2) {
            qtemp_idx_2 = q1_inv_idx_2 / C;
            vTheta = vTheta * qtemp_idx_2 * qtemp_idx_2 + 1.0;
            q1_inv_idx_2 = C;
        } else {
            qtemp_idx_2 = C / q1_inv_idx_2;
            vTheta += qtemp_idx_2 * qtemp_idx_2;
        }

        qNormalize_idx_1 = qtemp_idx_1 * qNormalize_idx_3 * t;
        C = fabs(qNormalize_idx_1);
        if (C > q1_inv_idx_2) {
            qtemp_idx_2 = q1_inv_idx_2 / C;
            vTheta = vTheta * qtemp_idx_2 * qtemp_idx_2 + 1.0;
            q1_inv_idx_2 = C;
        } else {
            qtemp_idx_2 = C / q1_inv_idx_2;
            vTheta += qtemp_idx_2 * qtemp_idx_2;
        }

        vTheta = q1_inv_idx_2 * sqrt(vTheta);
        if (vTheta != 0.0) {
            C = sin(vTheta);
            qtemp_idx_0 = cos(vTheta);
            qtemp_idx_1 = C * (absxk_tmp / vTheta);
            qtemp_idx_2 = C * (qNormalize_idx_2 / vTheta);
            C *= qNormalize_idx_1 / vTheta;
        } else {
            qtemp_idx_0 = 1.0;
            qtemp_idx_1 = 0.0;
            qtemp_idx_2 = 0.0;
            C = 0.0;
        }

        q_out[0] = ((q1[0] * qtemp_idx_0 - q1[1] * qtemp_idx_1) - q1[2] * qtemp_idx_2) - q1[3] * C;
        q_out[1] = (q1[0] * qtemp_idx_1 + qtemp_idx_0 * q1[1]) + (q1[2] * C - q1[3] * qtemp_idx_2);
        q_out[2] = (q1[0] * qtemp_idx_2 + qtemp_idx_0 * q1[2]) + (q1[3] * qtemp_idx_1 - q1[1] * C);
        q_out[3] = (q1[0] * C + qtemp_idx_0 * q1[3]) + (q1[1] * qtemp_idx_2 - q1[2] * qtemp_idx_1);

        /* q_out = quatnormalize(q_out); */
    }
}

void ArmMathFun::quat_squad(double q[8], double s, double val[4])
{
    double alpha;
    double C;
    double qtemp[4];
    double b_q[4];

    val[0] = q[0];
    val[1] = q[1];
    val[2] = q[2];
    val[3] = q[3];
    if (((q[0] * q[4] + q[1] * q[5]) + q[2] * q[6]) + q[3] * q[7] < 0.0) {
        q[4] = -q[4];
        q[5] = -q[5];
        q[6] = -q[6];
        q[7] = -q[7];
    }

    if (!(s == 0.0)) {
        if (s == 1.0) {
            val[0] = q[4];
            val[1] = q[5];
            val[2] = q[6];
            val[3] = q[7];
        } else {
            if ((2.0 >= s + 1.0) && (2.0 < (s + 1.0) + 1.0)) {
                alpha = (s + 1.0) - 1.0;
            } else {
                alpha = 0.0;
            }

            if (alpha > 0.0) {
                C = ((q[0] * q[4] + q[1] * q[5]) + q[2] * q[6]) + q[3] * q[7];
                if (1.0 - C <= 1.0E-9) {
                    C = q[0] * (1.0 - s) + q[4] * s;
                    val[0] = C;
                    qtemp[0] = C * C;
                    C = q[1] * (1.0 - s) + q[5] * s;
                    val[1] = C;
                    qtemp[1] = C * C;
                    C = q[2] * (1.0 - s) + q[6] * s;
                    val[2] = C;
                    qtemp[2] = C * C;
                    C = q[3] * (1.0 - s) + q[7] * s;
                    alpha = sqrt(((qtemp[0] + qtemp[1]) + qtemp[2]) + C * C);
                    val[0] /= alpha;
                    val[1] /= alpha;
                    val[2] /= alpha;
                    val[3] = C / alpha;
                } else {
                    if (C + 1.0 <= 1.0E-9) {
                        qtemp[2] = q[5];
                        qtemp[3] = -q[4];
                        q[4] = q[7];
                        q[5] = -q[6];
                        q[6] = qtemp[2];
                        q[7] = qtemp[3];
                    }

                    b_q[0] = q[4];
                    b_q[1] = q[5];
                    b_q[2] = q[6];
                    b_q[3] = q[7];
                    do_slerp(*(double (*)[4]) & q[0], b_q, alpha, qtemp);
                    b_q[0] = qtemp[0];
                    b_q[1] = qtemp[1];
                    b_q[2] = qtemp[2];
                    b_q[3] = qtemp[3];
                    do_slerp(qtemp, b_q, 2.0 * alpha * (1.0 - alpha), val);
                    alpha = sqrt(((val[0] * val[0] + val[1] * val[1]) + val[2] * val[2]) + val[3] * val[3]);
                    val[0] /= alpha;
                    val[1] /= alpha;
                    val[2] /= alpha;
                    val[3] /= alpha;
                }
            }
        }
    }
}

ArmMathFun::ArmMathFun() { rt_InitInfAndNaN(); }

void ArmMathFun::rt_InitInfAndNaN()
{
    rtNaN = std::numeric_limits<double>::quiet_NaN();
    rtNaNF = std::numeric_limits<double>::quiet_NaN();
    rtInf = std::numeric_limits<double>::infinity();
    rtInfF = std::numeric_limits<double>::infinity();
    rtMinusInf = -std::numeric_limits<double>::infinity();
    rtMinusInfF = -std::numeric_limits<double>::infinity();
}

bool ArmMathFun::rtIsInf(double value) { return ((value == rtInf || value == rtMinusInf) ? true : false); }

bool ArmMathFun::rtIsInfF(double value) { return (((value) == rtInfF || (value) == rtMinusInfF) ? true : false); }

bool ArmMathFun::rtIsNaN(double value) { return ((value != value) ? true : false); }

bool ArmMathFun::rtIsNaNF(double value) { return ((value != value) ? true : false); }

static double xnrm2(int n, const double x[9], int ix0)
{
    double y;
    double scale;
    int kend;
    int k;
    double absxk;
    double t;
    y = 0.0F;
    scale = 1.29246971E-26F;
    kend = (ix0 + n) - 1;  // todo 容易越界
    for (k = ix0; k <= kend; k++) {
        absxk = std::fabs(x[k - 1]);
        if (absxk > scale) {
            t = scale / absxk;
            y = y * t * t + 1.0F;
            scale = absxk;
        } else {
            t = absxk / scale;
            y += t * t;
        }
    }

    return scale * std::sqrt(y);
}

static void xaxpy(int n, double a, int ix0, double y[9], int iy0)
{
    int ix;
    int iy;
    int i;
    int k;
    if (!(a == 0.0F)) {
        ix = ix0 - 1;
        iy = iy0 - 1;
        i = n - 1;
        for (k = 0; k <= i; k++) {
            y[iy] += a * y[ix];
            ix++;
            iy++;
        }
    }
}

static double xdotc(int n, const double x[9], int ix0, const double y[9], int iy0)
{
    double d;
    int ix;
    int iy;
    int k;
    ix = ix0;
    iy = iy0;
    d = 0.0F;
    for (k = 0; k < n; k++) {
        d += x[ix - 1] * y[iy - 1];
        ix++;
        iy++;
    }

    return d;
}

static double b_xnrm2(const double x[3], int ix0)
{
    double y;
    double scale;
    int kend;
    int k;
    double absxk;
    double t;
    y = 0.0F;
    scale = 1.29246971E-26F;
    kend = ix0 + 1;
    for (k = ix0; k <= kend; k++) {
        absxk = std::fabs(x[k - 1]);
        if (absxk > scale) {
            t = scale / absxk;
            y = y * t * t + 1.0F;
            scale = absxk;
        } else {
            t = absxk / scale;
            y += t * t;
        }
    }

    return scale * std::sqrt(y);
}

static void b_xaxpy(int n, double a, const double x[9], int ix0, double y[3], int iy0)
{
    int ix;
    int iy;
    int i;
    int k;
    if (!(a == 0.0F)) {
        ix = ix0 - 1;
        iy = iy0 - 1;
        i = n - 1;
        for (k = 0; k <= i; k++) {
            y[iy] += a * x[ix];
            ix++;
            iy++;
        }
    }
}

static void c_xaxpy(int n, double a, const double x[3], int ix0, double y[9], int iy0)
{
    int ix;
    int iy;
    int i;
    int k;
    if (!(a == 0.0F)) {
        ix = ix0 - 1;
        iy = iy0 - 1;
        i = n - 1;
        for (k = 0; k <= i; k++) {
            y[iy] += a * x[ix];
            ix++;
            iy++;
        }
    }
}

static void xrotg(double* a, double* b, double* c, double* s)
{
    double roe;
    double absa;
    double absb;
    double scale;
    double ads;
    double bds;
    roe = *b;
    absa = std::fabs(*a);
    absb = std::fabs(*b);
    if (absa > absb) {
        roe = *a;
    }

    scale = absa + absb;
    if (scale == 0.0F) {
        *s = 0.0F;
        *c = 1.0F;
        *a = 0.0F;
        *b = 0.0F;
    } else {
        ads = absa / scale;
        bds = absb / scale;
        scale *= std::sqrt(ads * ads + bds * bds);
        if (roe < 0.0F) {
            scale = -scale;
        }

        *c = *a / scale;
        *s = *b / scale;
        if (absa > absb) {
            *b = *s;
        } else if (*c != 0.0F) {
            *b = 1.0F / *c;
        } else {
            *b = 1.0F;
        }

        *a = scale;
    }
}

static void xrot(double x[9], int ix0, int iy0, double c, double s)
{
    int ix;
    int iy;
    double temp;
    ix = ix0 - 1;
    iy = iy0 - 1;
    temp = c * x[ix] + s * x[iy];
    x[iy] = c * x[iy] - s * x[ix];
    x[ix] = temp;
    iy++;
    ix++;
    temp = c * x[ix] + s * x[iy];
    x[iy] = c * x[iy] - s * x[ix];
    x[ix] = temp;
    iy++;
    ix++;
    temp = c * x[ix] + s * x[iy];
    x[iy] = c * x[iy] - s * x[ix];
    x[ix] = temp;
}

void ArmMathFun::xswap(double x[9], int ix0, int iy0)
{
    int ix;
    int iy;
    double temp;
    ix = ix0 - 1;
    iy = iy0 - 1;
    temp = x[ix];
    x[ix] = x[iy];
    x[iy] = temp;
    ix++;
    iy++;
    temp = x[ix];
    x[ix] = x[iy];
    x[iy] = temp;
    ix++;
    iy++;
    temp = x[ix];
    x[ix] = x[iy];
    x[iy] = temp;
}

void ArmMathFun::svd(const double A[9], double U[9], double s[3], double V[9])
{
    double e[3];
    double work[3];
    int kase;
    bool apply_transform;
    double b_A[9];
    double nrm;
    double b_s[3];
    int qp1;
    int qjj;
    double r;
    int m;
    int q;
    int qq;
    double snorm;
    int exitg1;
    bool exitg2;
    double scale;
    double sm;
    double sqds;
    double b;
    e[0] = 0.0F;
    work[0] = 0.0F;
    e[1] = 0.0F;
    work[1] = 0.0F;
    e[2] = 0.0F;
    work[2] = 0.0F;
    for (kase = 0; kase < 9; kase++) {
        b_A[kase] = A[kase];
        U[kase] = 0.0F;
        V[kase] = 0.0F;
    }

    apply_transform = false;
    nrm = xnrm2(3, b_A, 1);
    if (nrm > 0.0F) {
        apply_transform = true;
        if (b_A[0] < 0.0F) {
            nrm = -nrm;
        }

        if (std::fabs(nrm) >= 9.86076132E-32F) {
            r = 1.0F / nrm;
            for (qp1 = 1; qp1 < 4; qp1++) {
                b_A[qp1 - 1] *= r;
            }
        } else {
            for (qp1 = 1; qp1 < 4; qp1++) {
                b_A[qp1 - 1] /= nrm;
            }
        }

        b_A[0]++;
        b_s[0] = -nrm;
    } else {
        b_s[0] = 0.0F;
    }

    for (kase = 2; kase < 4; kase++) {
        qjj = 3 * (kase - 1);
        if (apply_transform) {
            xaxpy(3, -(xdotc(3, b_A, 1, b_A, qjj + 1) / b_A[0]), 1, b_A, qjj + 1);
        }

        e[kase - 1] = b_A[qjj];
    }

    for (qp1 = 1; qp1 < 4; qp1++) {
        U[qp1 - 1] = b_A[qp1 - 1];
    }

    nrm = b_xnrm2(e, 2);
    if (nrm == 0.0F) {
        e[0] = 0.0F;
    } else {
        if (e[1] < 0.0F) {
            e[0] = -nrm;
        } else {
            e[0] = nrm;
        }

        r = e[0];
        if (std::fabs(e[0]) >= 9.86076132E-32F) {
            r = 1.0F / e[0];
            for (qp1 = 2; qp1 < 4; qp1++) {
                e[qp1 - 1] *= r;
            }
        } else {
            for (qp1 = 2; qp1 < 4; qp1++) {
                e[qp1 - 1] /= r;
            }
        }

        e[1]++;
        e[0] = -e[0];
        for (qp1 = 2; qp1 < 4; qp1++) {
            work[qp1 - 1] = 0.0F;
        }

        for (kase = 2; kase < 4; kase++) {
            b_xaxpy(2, e[kase - 1], b_A, 3 * (kase - 1) + 2, work, 2);
        }

        for (kase = 2; kase < 4; kase++) {
            c_xaxpy(2, -e[kase - 1] / e[1], work, 2, b_A, 3 * (kase - 1) + 2);
        }
    }

    for (qp1 = 2; qp1 < 4; qp1++) {
        V[qp1 - 1] = e[qp1 - 1];
    }

    apply_transform = false;
    nrm = xnrm2(2, b_A, 5);
    if (nrm > 0.0F) {
        apply_transform = true;
        if (b_A[4] < 0.0F) {
            nrm = -nrm;
        }

        if (std::fabs(nrm) >= 9.86076132E-32F) {
            r = 1.0F / nrm;
            for (qp1 = 5; qp1 < 7; qp1++) {
                b_A[qp1 - 1] *= r;
            }
        } else {
            for (qp1 = 5; qp1 < 7; qp1++) {
                b_A[qp1 - 1] /= nrm;
            }
        }

        b_A[4]++;
        b_s[1] = -nrm;
    } else {
        b_s[1] = 0.0F;
    }

    for (kase = 3; kase < 4; kase++) {
        if (apply_transform) {
            xaxpy(2, -(xdotc(2, b_A, 5, b_A, 8) / b_A[4]), 5, b_A, 8);
        }
    }

    for (qp1 = 2; qp1 < 4; qp1++) {
        U[qp1 + 2] = b_A[qp1 + 2];
    }

    m = 1;
    b_s[2] = b_A[8];
    e[1] = b_A[7];
    e[2] = 0.0F;
    U[6] = 0.0F;
    U[7] = 0.0F;
    U[8] = 1.0F;
    for (q = 1; q >= 0; q--) {
        qp1 = q + 2;
        qq = q + 3 * q;
        if (b_s[q] != 0.0F) {
            for (kase = qp1; kase < 4; kase++) {
                qjj = (q + 3 * (kase - 1)) + 1;
                xaxpy(3 - q, -(xdotc(3 - q, U, qq + 1, U, qjj) / U[qq]), qq + 1, U, qjj);
            }

            for (qp1 = q + 1; qp1 < 4; qp1++) {
                kase = (qp1 + 3 * q) - 1;
                U[kase] = -U[kase];
            }

            U[qq]++;
            if (0 <= q - 1) {
                U[3 * q] = 0.0F;
            }
        } else {
            U[3 * q] = 0.0F;
            U[3 * q + 1] = 0.0F;
            U[3 * q + 2] = 0.0F;
            U[qq] = 1.0F;
        }
    }

    for (q = 2; q >= 0; q--) {
        if ((q + 1 <= 1) && (e[0] != 0.0F)) {
            xaxpy(2, -(xdotc(2, V, 2, V, 5) / V[1]), 2, V, 5);
            xaxpy(2, -(xdotc(2, V, 2, V, 8) / V[1]), 2, V, 8);
        }

        V[3 * q] = 0.0F;
        V[3 * q + 1] = 0.0F;
        V[3 * q + 2] = 0.0F;
        V[q + 3 * q] = 1.0F;
    }

    qq = 0;
    snorm = 0.0F;
    for (q = 0; q < 3; q++) {
        if (b_s[q] != 0.0F) {
            nrm = std::fabs(b_s[q]);
            r = b_s[q] / nrm;
            b_s[q] = nrm;
            if (q + 1 < 3) {
                e[q] /= r;
            }

            qjj = 3 * q;
            kase = qjj + 3;
            for (qp1 = qjj + 1; qp1 <= kase; qp1++) {
                U[qp1 - 1] *= r;
            }
        }

        if ((q + 1 < 3) && (e[q] != 0.0F)) {
            nrm = std::fabs(e[q]);
            r = nrm / e[q];
            e[q] = nrm;
            b_s[q + 1] *= r;
            qjj = 3 * (q + 1);
            kase = qjj + 3;
            for (qp1 = qjj + 1; qp1 <= kase; qp1++) {
                V[qp1 - 1] *= r;
            }
        }

        nrm = std::fabs(b_s[q]);
        r = std::fabs(e[q]);
        if ((nrm > r) || ArmMathFun::rtIsNaNF(r)) {
            r = nrm;
        }

        if ((!(snorm > r)) && (!ArmMathFun::rtIsNaNF(r))) {
            snorm = r;
        }
    }

    while ((m + 2 > 0) && (qq < 75)) {
        qp1 = m;
        do {
            exitg1 = 0;
            q = qp1 + 1;
            if (qp1 + 1 == 0) {
                exitg1 = 1;
            } else {
                nrm = std::fabs(e[qp1]);
                if ((nrm <= 1.1920929E-7F * (std::fabs(b_s[qp1]) + std::fabs(b_s[qp1 + 1]))) || (nrm <= 9.86076132E-32F) || ((qq > 20) && (nrm <= 1.1920929E-7F * snorm))) {
                    e[qp1] = 0.0F;
                    exitg1 = 1;
                } else {
                    qp1--;
                }
            }
        } while (exitg1 == 0);

        if (qp1 + 1 == m + 1) {
            kase = 4;
        } else {
            qjj = m + 2;
            kase = m + 2;
            exitg2 = false;
            while ((!exitg2) && (kase >= qp1 + 1)) {
                qjj = kase;
                if (kase == qp1 + 1) {
                    exitg2 = true;
                } else {
                    nrm = 0.0F;
                    if (kase < m + 2) {
                        nrm = std::fabs(e[kase - 1]);
                    }

                    if (kase > qp1 + 2) {
                        nrm += std::fabs(e[kase - 2]);
                    }

                    r = std::fabs(b_s[kase - 1]);
                    if ((r <= 1.1920929E-7F * nrm) || (r <= 9.86076132E-32F)) {
                        b_s[kase - 1] = 0.0F;
                        exitg2 = true;
                    } else {
                        kase--;
                    }
                }
            }

            if (qjj == qp1 + 1) {
                kase = 3;
            } else if (qjj == m + 2) {
                kase = 1;
            } else {
                kase = 2;
                q = qjj;
            }
        }

        switch (kase) {
            case 1:
                r = e[m];
                e[m] = 0.0F;
                kase = m + 1;
                for (qp1 = kase; qp1 >= q + 1; qp1--) {
                    xrotg(&b_s[qp1 - 1], &r, &sm, &sqds);
                    if (qp1 > q + 1) {
                        r = -sqds * e[0];
                        e[0] *= sm;
                    }

                    xrot(V, 3 * (qp1 - 1) + 1, 3 * (m + 1) + 1, sm, sqds);
                }
                break;

            case 2:
                r = e[q - 1];
                e[q - 1] = 0.0F;
                for (qp1 = q + 1; qp1 <= m + 2; qp1++) {
                    xrotg(&b_s[qp1 - 1], &r, &sm, &sqds);
                    b = e[qp1 - 1];
                    r = -sqds * b;
                    e[qp1 - 1] = b * sm;
                    xrot(U, 3 * (qp1 - 1) + 1, 3 * (q - 1) + 1, sm, sqds);
                }
                break;

            case 3:
                kase = m + 1;
                nrm = b_s[m + 1];
                scale = std::fabs(nrm);
                r = std::fabs(b_s[m]);
                if ((!(scale > r)) && (!rtIsNaNF(r))) {
                    scale = r;
                }

                r = std::fabs(e[m]);
                if ((!(scale > r)) && (!rtIsNaNF(r))) {
                    scale = r;
                }

                r = std::fabs(b_s[q]);
                if ((!(scale > r)) && (!rtIsNaNF(r))) {
                    scale = r;
                }

                r = std::fabs(e[q]);
                if ((!(scale > r)) && (!rtIsNaNF(r))) {
                    scale = r;
                }

                sm = nrm / scale;
                nrm = b_s[m] / scale;
                r = e[m] / scale;
                sqds = b_s[q] / scale;
                b = ((nrm + sm) * (nrm - sm) + r * r) / 2.0F;
                nrm = sm * r;
                nrm *= nrm;
                if ((b != 0.0F) || (nrm != 0.0F)) {
                    r = std::sqrt(b * b + nrm);
                    if (b < 0.0F) {
                        r = -r;
                    }

                    r = nrm / (b + r);
                } else {
                    r = 0.0F;
                }

                r += (sqds + sm) * (sqds - sm);
                nrm = sqds * (e[q] / scale);
                for (qp1 = q + 1; qp1 <= kase; qp1++) {
                    xrotg(&r, &nrm, &sm, &sqds);
                    if (qp1 > q + 1) {
                        e[0] = r;
                    }

                    nrm = e[qp1 - 1];
                    b = b_s[qp1 - 1];
                    e[qp1 - 1] = sm * nrm - sqds * b;
                    r = sqds * b_s[qp1];
                    b_s[qp1] *= sm;
                    xrot(V, 3 * (qp1 - 1) + 1, 3 * qp1 + 1, sm, sqds);
                    b_s[qp1 - 1] = sm * b + sqds * nrm;
                    xrotg(&b_s[qp1 - 1], &r, &sm, &sqds);
                    r = sm * e[qp1 - 1] + sqds * b_s[qp1];
                    b_s[qp1] = -sqds * e[qp1 - 1] + sm * b_s[qp1];
                    nrm = sqds * e[qp1];
                    e[qp1] *= sm;
                    xrot(U, 3 * (qp1 - 1) + 1, 3 * qp1 + 1, sm, sqds);
                }

                e[m] = r;
                qq++;
                break;

            default:
                if (b_s[q] < 0.0F) {
                    b_s[q] = -b_s[q];
                    qjj = 3 * q;
                    kase = qjj + 3;
                    for (qp1 = qjj + 1; qp1 <= kase; qp1++) {
                        V[qp1 - 1] = -V[qp1 - 1];
                    }
                }

                qp1 = q + 1;
                while ((q + 1 < 3) && (b_s[q] < b_s[qp1])) {
                    nrm = b_s[q];
                    b_s[q] = b_s[qp1];
                    b_s[qp1] = nrm;
                    xswap(V, 3 * q + 1, 3 * (q + 1) + 1);
                    xswap(U, 3 * q + 1, 3 * (q + 1) + 1);
                    q = qp1;
                    qp1++;
                }

                qq = 0;
                m--;
                break;
        }
    }

    s[0] = b_s[0];
    s[1] = b_s[1];
    s[2] = b_s[2];
}

// Vec3<double> ArmMathFun::rotMat2rotVec(Mat3<double> R)
// {
//     Vec3<double> rVec;
//     int r;
//     bool p;
//     double rotationMatrix[9];
//     int k;
//     double U[9];
//     double s[3];
//     double V[9];
//     double t;
//     double absxk;
//     double theta;
//     double b_s;
//     int idx;
//     bool exitg1;
//     int iidx;
//     double y;
//     // 由于构造函数mathfun时，已经初始化数值计算的边界值，奇异值，因此这语句不再需要，注意！
//     /*
//     if (isInitialized_rotMat2rotVec == false) {
//         rotMat2rotVec_initialize();
//     }
//     */
//     for (r = 0; r < 3; r++) {
//         // rotationMatrix[3 * r] = R[r];
//         // rotationMatrix[3 * r + 1] = R[r + 3];
//         // rotationMatrix[3 * r + 2] = R[r + 6];

//         rotationMatrix[3 * r] = R(r, 0);
//         rotationMatrix[3 * r + 1] = R(r, 1);
//         rotationMatrix[3 * r + 2] = R(r, 2);
//     }

//     p = true;
//     for (k = 0; k < 9; k++) {
//         if ((!p) || (rtIsInfF(rotationMatrix[k]) || rtIsNaNF(rotationMatrix[k]))) {
//             p = false;
//         }
//     }

//     if (p) {
//         svd(rotationMatrix, U, s, V);
//     } else {
//         for (r = 0; r < 9; r++) {
//             U[r] = rtNaNF;
//             V[r] = rtNaNF;
//         }
//     }

//     for (r = 0; r < 3; r++) {
//         absxk = U[r + 3];
//         b_s = U[r + 6];
//         for (idx = 0; idx < 3; idx++) {
//             rotationMatrix[r + 3 * idx] = (U[r] * V[idx] + absxk * V[idx + 3]) + b_s * V[idx + 6];
//         }
//     }

//     t = (rotationMatrix[0] + rotationMatrix[4]) + rotationMatrix[8];
//     theta = std::acos((t - 1.0F) / 2.0F);
//     rVec(0) = rotationMatrix[5] - rotationMatrix[7];
//     rVec(1) = rotationMatrix[6] - rotationMatrix[2];
//     rVec(2) = rotationMatrix[1] - rotationMatrix[3];
//     absxk = std::sin(theta);
//     if (absxk >= 0.0001F) {
//         b_s = 1.0F / (2.0F * absxk);
//         rVec(0) = theta * (rVec(0) * b_s);
//         rVec(1) = theta * (rVec(1) * b_s);
//         rVec(2) = theta * (rVec(2) * b_s);
//     } else if (t - 1.0F > 0.0F) {
//         b_s = (t - 3.0F) / 12.0F;
//         rVec(0) *= 0.5F - b_s;
//         rVec(1) *= 0.5F - b_s;
//         rVec(2) *= 0.5F - b_s;
//     } else {
//         s[0] = rotationMatrix[0];
//         s[1] = rotationMatrix[4];
//         s[2] = rotationMatrix[8];
//         if (!rtIsNaNF(rotationMatrix[0])) {
//             idx = 1;
//         } else {
//             idx = 0;
//             k = 2;
//             exitg1 = false;
//             while ((!exitg1) && (k < 4)) {
//                 if (!rtIsNaNF(s[k - 1])) {
//                     idx = k;
//                     exitg1 = true;
//                 } else {
//                     k++;
//                 }
//             }
//         }

//         if (idx == 0) {
//             iidx = 0;
//         } else {
//             b_s = s[idx - 1];
//             iidx = idx - 1;
//             r = idx + 1;
//             for (k = r; k < 4; k++) {
//                 absxk = s[k - 1];
//                 if (b_s < absxk) {
//                     b_s = absxk;
//                     iidx = k - 1;
//                 }
//             }
//         }

//         idx = iidx + 1;
//         r = static_cast<int>(std::fmod(static_cast<double>(idx), 3.0));
//         idx = static_cast<int>(std::fmod(static_cast<double>(idx) + 1.0, 3.0));
//         b_s = std::sqrt(((rotationMatrix[iidx + 3 * iidx] - rotationMatrix[r + 3 * r]) - rotationMatrix[idx + 3 * idx]) + 1.0F);
//         rVec(0) = 0.0F;
//         rVec(1) = 0.0F;
//         rVec(2) = 0.0F;
//         rVec(iidx) = b_s / 2.0F;
//         rVec(r) = (rotationMatrix[r + 3 * iidx] + rotationMatrix[iidx + 3 * r]) / (2.0F * b_s);
//         rVec(idx) = (rotationMatrix[idx + 3 * iidx] + rotationMatrix[iidx + 3 * idx]) / (2.0F * b_s);
//         b_s = 1.29246971E-26F;
//         absxk = std::fabs(rVec(0));
//         if (absxk > 1.29246971E-26F) {
//             y = 1.0F;
//             b_s = absxk;
//         } else {
//             t = absxk / 1.29246971E-26F;
//             y = t * t;
//         }

//         absxk = std::fabs(rVec(1));
//         if (absxk > b_s) {
//             t = b_s / absxk;
//             y = y * t * t + 1.0F;
//             b_s = absxk;
//         } else {
//             t = absxk / b_s;
//             y += t * t;
//         }

//         absxk = std::fabs(rVec(2));
//         if (absxk > b_s) {
//             t = b_s / absxk;
//             y = y * t * t + 1.0F;
//             b_s = absxk;
//         } else {
//             t = absxk / b_s;
//             y += t * t;
//         }

//         y = b_s * std::sqrt(y);
//         rVec(0) = theta * rVec(0) / y;
//         rVec(1) = theta * rVec(1) / y;
//         rVec(2) = theta * rVec(2) / y;
//     }
//     return rVec;
// }

bool ArmMathFun::IsOdd(int u)
{
    if (u % 2 == 0) {
        return true;
    } else {
        return false;
    }
}

// Mat3<double> ArmMathFun::quat2RotMat(Vec4<double> Vec)
// {
//     // quat = w x y z
//     double e0 = Vec(0);
//     double e1 = Vec(1);
//     double e2 = Vec(2);
//     double e3 = Vec(3);

//     Mat3<double> R;
//     R << 1 - 2 * (e2 * e2 + e3 * e3), 2 * (e1 * e2 - e0 * e3),
//         2 * (e1 * e3 + e0 * e2), 2 * (e1 * e2 + e0 * e3),
//         1 - 2 * (e1 * e1 + e3 * e3), 2 * (e2 * e3 - e0 * e1),
//         2 * (e1 * e3 - e0 * e2), 2 * (e2 * e3 + e0 * e1),
//         1 - 2 * (e1 * e1 + e2 * e2);
//     return R;
// }

Vec3<double> ArmMathFun::quat2RPY(Vec4<double> q)
{
    Vec3<double> rpy;
    double as = min(-2. * (q[1] * q[3] - q[0] * q[2]), .99999);
    rpy(2) = atan2(2 * (q[1] * q[2] + q[0] * q[3]), pow(q[0], 2) + pow(q[1], 2) - pow(q[2], 2) - pow(q[3], 2));
    rpy(1) = asin(as);
    rpy(0) = atan2(2 * (q[2] * q[3] + q[0] * q[1]), pow(q[0], 2) - pow(q[1], 2) - pow(q[2], 2) + pow(q[3], 2));
    return rpy;
}
/**
 * @brief 通用的机械臂正运动学
 *
 * @param Radian 关节角度
 * @return Mat4<double>
 */
Pose ArmMathFun::Fkine(Vec6<double> Radian, Pose ToolValue)
{
    auto& armParam = GetArmParam();
    Mat4<double> Tbase, Ttool, KPS44, T1, T2, T3, T4, T5, T6, T7;
    vector<double> d, a, alpha, offset, Q;
    Vec6<double> sq, cq, sa, ca;
    Pose KPS6;
    Tbase = ArmMathFun::Cartesian2RotMat(armParam.base.base);
    Ttool = ArmMathFun::Cartesian2RotMat(armParam.tool.tool);
    d = armParam.kine_param.d;
    a = armParam.kine_param.a;
    alpha = armParam.kine_param.alpha;
    // offset = armParam.kine_param.offset;
    for (int i = 0; i < 6; i++) {
        Q.push_back(Radian(i) + armParam.kine_param.offset[i]);
    }

    auto Usertool = ArmMathFun::Cartesian2RotMat(ToolValue);

    for (int i = 0; i < 6; i++) {
        sq(i) = sin(Q[i]);
        cq(i) = cos(Q[i]);
        sa(i) = sin(alpha[i]);
        ca(i) = cos(alpha[i]);
    }

    T1 << cq(0), -sq(0), 0, a[0], sq(0) * ca(0), cq(0) * ca(0), -sa(0), -sa(0) * d[0], sq(0) * sa(0), cq(0) * sa(0), ca(0), ca(0) * d[0], 0, 0, 0, 1;
    T2 << cq(1), -sq(1), 0, a[1], sq(1) * ca(1), cq(1) * ca(1), -sa(1), -sa(1) * d[1], sq(1) * sa(1), cq(1) * sa(1), ca(1), ca(1) * d[1], 0, 0, 0, 1;
    T3 << cq(2), -sq(2), 0, a[2], sq(2) * ca(2), cq(2) * ca(2), -sa(2), -sa(2) * d[2], sq(2) * sa(2), cq(2) * sa(2), ca(2), ca(2) * d[2], 0, 0, 0, 1;
    T4 << cq(3), -sq(3), 0, a[3], sq(3) * ca(3), cq(3) * ca(3), -sa(3), -sa(3) * d[3], sq(3) * sa(3), cq(3) * sa(3), ca(3), ca(3) * d[3], 0, 0, 0, 1;
    T5 << cq(4), -sq(4), 0, a[4], sq(4) * ca(4), cq(4) * ca(4), -sa(4), -sa(4) * d[4], sq(4) * sa(4), cq(4) * sa(4), ca(4), ca(4) * d[4], 0, 0, 0, 1;
    T6 << cq(5), -sq(5), 0, a[5], sq(5) * ca(5), cq(5) * ca(5), -sa(5), -sa(5) * d[5], sq(5) * sa(5), cq(5) * sa(5), ca(5), ca(5) * d[5], 0, 0, 0, 1;

    KPS44 = Tbase * T1 * T2 * T3 * T4 * T5 * T6 * Ttool * Usertool;

    KPS6 = ArmMathFun::RotMat2Cartesian(KPS44);

    return KPS6;
}

/**
 * @brief 满足Pieper准则设计的机械臂逆运动学解析解
 *
 * @param KPS44 位姿信息
 * @param qReward 上一次的关节位置
 * @return Vec6<double>
 */
pair<bool, Vec6<double>> ArmMathFun::Ikine(Pose KPS6, Vec6<double> qReward, Pose ToolValue)
{
    auto& armParam = GetArmParam();

    double s1, s3, s4, s5, s23, c1, c3, c4, c5, c23, K = 0, ss = 0, zyj = 8, sss1 = 0, sss2 = 0, ssss1 = 0, ssss2 = 0, sssss1 = 0, sssss2 = 0, h = 0, Min_ = 0;
    double x, y, z, nx, ny, nz, ax, ay, az;
    Mat4<double> KPS44, T_base, T_tool, inv_Tbase, inv_Ttool, R44, Tool1;
    Vec2<double> Q1;
    Vec4<double> Q3;
    Mat2<double> Q2, Q23, Q4, Q5, Q6;
    Mat86<double> QQ;

    double d4, a2, a3, offset2, offset3, offset5;
    Vec6<double> Qmax, Qmin, new_q;
    Vec8<double> bz, Min;

    pair<bool, Vec6<double>> IK;
    IK.first = false;
    bz.setZero();
    Min.setZero();

    d4 = armParam.kine_param.d[3];
    a2 = armParam.kine_param.a[2];
    a3 = armParam.kine_param.a[3];

    offset2 = armParam.kine_param.offset[1];
    offset3 = armParam.kine_param.offset[2];
    offset5 = armParam.kine_param.offset[4];
    for (size_t i = 0; i < armParam.axis.S_max_real.size(); i++) {
        Qmax(i) = armParam.axis.S_max_real[i];
        Qmin(i) = armParam.axis.S_min_real[i];
    }

    T_base = ArmMathFun::Cartesian2RotMat(armParam.base.base);
    T_tool = ArmMathFun::Cartesian2RotMat(armParam.tool.tool);

    auto Usertool = ArmMathFun::Cartesian2RotMat(ToolValue);

    KPS44 = ArmMathFun::Cartesian2RotMat(KPS6);

    Q1.fill(0);
    Q2.fill(0);
    Q3.fill(0);
    Q4.fill(0);
    Q5.fill(0);
    Q6.fill(0);
    Q23.fill(0);
    QQ.fill(0);

    T_tool = T_tool * Usertool;

    inv_Tbase = T_base.inverse();
    inv_Ttool = T_tool.inverse();

    R44 = inv_Tbase * KPS44 * inv_Ttool;

    qReward(1) = qReward(1) - offset2;
    qReward(2) = qReward(2) - offset3;
    qReward(4) = qReward(4) - offset5;

    x = R44(0, 3);
    y = R44(1, 3);
    z = R44(2, 3);
    nx = R44(0, 0);
    ny = R44(1, 0);
    nz = R44(2, 0);
    ax = R44(0, 2);
    ay = R44(1, 2);
    az = R44(2, 2);

    Q1(0) = atan2(y, x);
    Q1(1) = atan2(-y, -x);

    K = (pow(x, 2) + pow(y, 2) + pow(z, 2) - pow(a2, 2) - pow(a3, 2) - pow(d4, 2)) / (2 * a2);

    if (pow(a3, 2) + pow(d4, 2) - pow(K, 2) >= 0) {
        ss = sqrt(pow(a3, 2) + pow(d4, 2) - pow(K, 2));
        Q3(0) = atan2(a3, d4) - atan2(K, ss);
        Q3(1) = atan2(a3, d4) - atan2(K, -ss);
    } else {
        Q3(0) = 10000;
        Q3(1) = 10000;
    }

    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
            if (i == 1) {
                Q3(2) = Q3(0);
                Q3(3) = Q3(1);
            }
            s1 = sin(Q1(i));
            s3 = sin(Q3(j));
            c1 = cos(Q1(i));
            c3 = cos(Q3(j));
            Q23(i, j) = atan2((-a3 - a2 * c3) * z - (c1 * R44(0, 3) + s1 * y) * (d4 - a2 * s3), (a2 * s3 - d4) * z + (a3 + a2 * c3) * (c1 * x + s1 * y));
            Q2(i, j) = Q23(i, j) - Q3(j);
            s23 = sin(Q23(i, j));
            c23 = cos(Q23(i, j));
            sss1 = -ax * s1 + ay * c1;
            sss2 = -ax * c1 * c23 - ay * s1 * c23 + az * s23;

            if (fabs(sss1) < 0.000000001 && fabs(sss2) < 0.000000001) {
                Q4(i, j) = qReward(3);
                Q5(i, j) = 0;
                s4 = sin(Q4(i, j));
                c4 = cos(Q4(i, j));
                s5 = sin(Q5(i, j));
                c5 = cos(Q5(i, j));
                sssss1 = -nx * (c1 * c23 * s4 - s1 * c4) - ny * (s1 * c23 * s4 + c1 * c4) + nz * (s23 * s4);
                sssss2 = nx * ((c1 * c23 * c4 + s1 * s4) * c5 - c1 * s23 * s5) + ny * ((s1 * c23 * c4 - c1 * s4) * c5 - s1 * s23 * s5) - nz * (s23 * c4 * c5 + c23 * s5);
                Q6(i, j) = atan2(sssss1, sssss2);
            } else {
                Q4(i, j) = atan2(sss1, sss2);
                s4 = sin(Q4(i, j));
                c4 = cos(Q4(i, j));

                ssss1 = -ax * (c1 * c23 * c4 + s1 * s4) - ay * (s1 * c23 * c4 - c1 * s4) + az * (s23 * c4);
                ssss2 = ax * (-c1 * s23) + ay * (-s1 * s23) + az * (-c23);
                Q5(i, j) = atan2(ssss1, ssss2);  // right
                s5 = sin(Q5(i, j));
                c5 = cos(Q5(i, j));
                sssss1 = -nx * (c1 * c23 * s4 - s1 * c4) - ny * (s1 * c23 * s4 + c1 * c4) + nz * (s23 * s4);
                sssss2 = nx * ((c1 * c23 * c4 + s1 * s4) * c5 - c1 * s23 * s5) + ny * ((s1 * c23 * c4 - c1 * s4) * c5 - s1 * s23 * s5) - nz * (s23 * c4 * c5 + c23 * s5);
                Q6(i, j) = atan2(sssss1, sssss2);
            }

            QQ(h, 0) = Q1(i);
            QQ(h, 1) = Q2(i, j) - offset2;
            QQ(h, 2) = Q3(j) - offset3;
            QQ(h, 3) = Q4(i, j);
            QQ(h, 4) = Q5(i, j) - offset5;
            QQ(h, 5) = Q6(i, j);
            QQ(h + 4, 0) = Q1(i);
            QQ(h + 4, 1) = Q2(i, j) - offset2;
            QQ(h + 4, 2) = Q3(j) - offset3;
            QQ(h + 4, 3) = Q4(i, j) + M_PI;
            QQ(h + 4, 4) = -Q5(i, j) - offset5;
            QQ(h + 4, 5) = Q6(i, j) + M_PI;
            h = h + 1;
        }
    }
    // cout << "QQ:" << QQ << endl;
    for (int m = 0; m < 8; m++) {
        for (int n = 0; n < 6; n++) {
            if (QQ(m, n) < Qmin(n)) {
                QQ(m, n) = QQ(m, n) + 2 * M_PI;
            }
            if (QQ(m, n) > Qmax(n)) {
                QQ(m, n) = QQ(m, n) - 2 * M_PI;
            }
            if (fabs(QQ(m, n) - qReward(n)) > 2 * M_PI) {
                if (QQ(m, n) >= 0) {
                    QQ(m, n) = QQ(m, n) - 2 * M_PI;
                } else {
                    QQ(m, n) = QQ(m, n) + 2 * M_PI;
                }
            }
        }
    }

    //      search_angle
    for (int i = 0; i < 8; i++) {
        if ((QQ(i, 0) >= Qmin(0)) && (QQ(i, 0 <= Qmax(0))) && (QQ(i, 1) >= Qmin(1)) && (QQ(i, 1) <= Qmax(1)) && (QQ(i, 2) >= Qmin(2)) && (QQ(i, 2) <= Qmax(2)) &&
            (QQ(i, 2) >= Qmin(3)) && (QQ(i, 3) <= Qmax(3)) && (QQ(i, 4) >= Qmin(4)) && (QQ(i, 4) <= Qmax(4)) && (QQ(i, 5) >= Qmin(5)) && (QQ(i, 5) <= Qmax(5))) {
            bz(i) = 1;  // ���н��־���?1�Ǳ�ʾ�ǿ��н�
            Min(i) = fabs(qReward(0) - QQ(i, 0)) + fabs(qReward(1) - QQ(i, 1)) + fabs(qReward(2) - QQ(i, 2)) + fabs(qReward(3) - QQ(i, 3)) + fabs(qReward(4) - QQ(i, 4)) +
                     fabs(qReward(5) - QQ(i, 5));  // �����Ӧ���ֵ
        } else {
            bz(i) = 0;
            Min(i) = 0;
        }
    }

    Min_ = Min(0) + Min(1) + Min(2) + Min(3) + Min(4) + Min(5) + Min(6) + Min(7);
    for (int i = 0; i < 8; i++) {
        if (bz(i) == 1 && Min_ >= Min(i)) {
            Min_ = Min(i);
            zyj = i;
        }
    }

    if (zyj == 8) {
        IK.first = false;
        qReward(1) = qReward(1) + offset2;
        qReward(2) = qReward(2) + offset3;
        qReward(4) = qReward(4) + offset5;
        for (int i = 0; i < 6; i++) {
            new_q(i) = qReward(i);
        }
    } else {  // best
        IK.first = true;
        for (int i = 0; i < 6; i++) {
            new_q(i) = QQ(zyj, i);
        }
    }
    IK.second = new_q;
    return IK;
}

/**
 * @brief 欧拉角转四元数
 * @param rpy
 * @return Quaternion
 */
Quaternion ArmMathFun::Rpy2quat(Rpy rpy)
{
    Quaternion quat;
    double sx = sin(rpy.rx / 2);
    double cx = cos(rpy.rx / 2);
    double sy = sin(rpy.ry / 2);
    double cy = cos(rpy.ry / 2);
    double sz = sin(rpy.rz / 2);
    double cz = cos(rpy.rz / 2);

    quat.w = cx * cy * cz + sx * sy * sz;
    quat.x = sx * cy * cz - cx * sy * sz;
    quat.y = cx * sy * cz + sx * cy * sz;
    quat.z = cx * cy * sz - sx * sy * cz;

    return quat;
}
/**
 * @brief 四元数转欧拉角
 *
 * @param quat
 * @return RPY
 */
Rpy ArmMathFun::Quat2rpy(Quaternion quat)
{
    Rpy rpy;
    rpy.rx = atan2(2 * (quat.w * quat.x + quat.y * quat.z), 1 - 2 * (pow(quat.x, 2) + pow(quat.y, 2)));
    rpy.ry = asin(2 * (quat.w * quat.y - quat.x * quat.z));
    rpy.rz = atan2(2 * (quat.w * quat.z + quat.x * quat.y), 1 - 2 * (pow(quat.y, 2) + pow(quat.z, 2)));

    return rpy;
}
/**
 * @brief 旋转矩阵转笛卡尔
 *
 * @param KPS44 旋转矩阵
 * @return Vec6<double>
 */
Pose ArmMathFun::RotMat2Cartesian(Mat4<double> KPS44)
{
    Pose pose;

    pose.tran_.x = KPS44(0, 3);
    pose.tran_.y = KPS44(1, 3);
    pose.tran_.z = KPS44(2, 3);
    pose.rpy_.ry = atan2(-KPS44(2, 0), sqrt(pow(KPS44(0, 0), 2) + pow(KPS44(1, 0), 2)));
    if (abs(abs(pose.rpy_.ry) - M_PI / 2.0) < 0.000001) {
        if (pose.rpy_.ry > 0) {
            pose.rpy_.rx = atan2(KPS44(0, 1), KPS44(1, 1));
            pose.rpy_.ry = 0.5 * M_PI;
            pose.rpy_.rz = 0.0;
        } else {
            pose.rpy_.rx = -atan2(KPS44(0, 1), KPS44(1, 1));
            pose.rpy_.ry = 0.5 * -M_PI;
            pose.rpy_.rz = 0.0;
        }
    } else {
        double cp = cos(pose.tran_.x);
        pose.rpy_.rz = atan2(KPS44(1, 0) / cp, KPS44(0, 0) / cp);
        pose.rpy_.rx = atan2(KPS44(2, 1) / cp, KPS44(2, 2) / cp);
    }

    return pose;
}
/**
 * @brief 欧拉角转旋转矩阵
 * 顺序是R= Rz*Ry*Rx
 * @param pos 位姿
 * @return Mat4<double>
 */
Mat4<double> ArmMathFun::Cartesian2RotMat(Pose pos)
{
    Mat4<double> KPS44;
    double sx = sin(pos.rpy_.rx);
    double cx = cos(pos.rpy_.rx);
    double sy = sin(pos.rpy_.ry);
    double cy = cos(pos.rpy_.ry);
    double sz = sin(pos.rpy_.rz);
    double cz = cos(pos.rpy_.rz);

    KPS44(0, 0) = cy * cz;
    KPS44(0, 1) = cz * sx * sy - cx * sz;
    KPS44(0, 2) = sx * sz + cx * cz * sy;
    KPS44(0, 3) = pos.tran_.x;
    KPS44(1, 0) = cy * sz;
    KPS44(1, 1) = cx * cz + sx * sy * sz;
    KPS44(1, 2) = cx * sy * sz - cz * sx;
    KPS44(1, 3) = pos.tran_.y;
    KPS44(2, 0) = -sy;
    KPS44(2, 1) = cy * sx;
    KPS44(2, 2) = cx * cy;
    KPS44(2, 3) = pos.tran_.z;
    KPS44(3, 0) = 0.0;
    KPS44(3, 1) = 0.0;
    KPS44(3, 2) = 0.0;
    KPS44(3, 3) = 1.0;

    return KPS44;
}

double ArmMathFun::CumputeQs(Quaternion quat_start, Quaternion quat_end)
{
    double quatS, costmp;
    Vec4<double> quat_start_inv, q0inv_q1, log_qs_inv_qe, qs_log_qs_inv_qe, qs_log_qs_inv_qe_qsinv;
    double tmp_ = 0.0;
    double a = 0.0;
    double tmp = 0;

    costmp = (quat_start.x * quat_end.x) + (quat_start.y * quat_end.y) + (quat_start.z * quat_end.z) + (quat_start.w * quat_end.w);

    if (costmp < 0) {
        quat_end.x = -quat_end.x;
        quat_end.y = -quat_end.y;
        quat_end.z = -quat_end.z;
        quat_end.w = -quat_end.w;
    }

    quat_start_inv << -quat_start.x, -quat_start.y, -quat_start.z, quat_start.w;

    q0inv_q1(0) = quat_start_inv(0) * quat_end.w + quat_start_inv(3) * quat_end.x + quat_start_inv(1) * quat_end.y - quat_start_inv(2) * quat_end.x;
    q0inv_q1(1) = quat_start_inv(1) * quat_end.w + quat_start_inv(3) * quat_end.y + quat_start_inv(2) * quat_end.x - quat_start_inv(0) * quat_end.z;
    q0inv_q1(2) = quat_start_inv(2) * quat_end.w + quat_start_inv(3) * quat_end.z + quat_start_inv(0) * quat_end.y - quat_start_inv(1) * quat_end.x;
    q0inv_q1(3) = quat_start_inv(3) * quat_end.w - quat_start_inv(0) * quat_end.x - quat_start_inv(1) * quat_end.y - quat_start_inv(2) * quat_end.z;

    tmp_ = pow(q0inv_q1(3), 2);
    if (tmp_ > 1) {
        tmp_ = 1;
    }

    a = acos(q0inv_q1(3));

    if (sqrt(1 - tmp_) > 0.0000000001) {
        tmp = a / sqrt(1 - tmp_);
    }

    log_qs_inv_qe(0) = q0inv_q1(0) * tmp;
    log_qs_inv_qe(1) = q0inv_q1(1) * tmp;
    log_qs_inv_qe(2) = q0inv_q1(2) * tmp;
    log_qs_inv_qe(3) = 0;

    qs_log_qs_inv_qe(0) = quat_start.x * log_qs_inv_qe(3) + quat_start.w * log_qs_inv_qe(0) + quat_start.y * log_qs_inv_qe(2) - quat_start.z * log_qs_inv_qe(1);
    qs_log_qs_inv_qe(1) = quat_start.y * log_qs_inv_qe(3) + quat_start.w * log_qs_inv_qe(1) + quat_start.z * log_qs_inv_qe(0) - quat_start.x * log_qs_inv_qe(2);
    qs_log_qs_inv_qe(2) = quat_start.z * log_qs_inv_qe(3) + quat_start.w * log_qs_inv_qe(2) + quat_start.x * log_qs_inv_qe(1) - quat_start.y * log_qs_inv_qe(0);
    qs_log_qs_inv_qe(3) = quat_start.w * log_qs_inv_qe(3) - quat_start.x * log_qs_inv_qe(0) - quat_start.y * log_qs_inv_qe(1) - quat_start.z * log_qs_inv_qe(2);

    qs_log_qs_inv_qe_qsinv(0) =
        qs_log_qs_inv_qe(0) * quat_start_inv(3) + qs_log_qs_inv_qe(3) * quat_start_inv(0) + qs_log_qs_inv_qe(1) * quat_start_inv(2) - qs_log_qs_inv_qe(2) * quat_start_inv(1);
    qs_log_qs_inv_qe_qsinv(1) =
        qs_log_qs_inv_qe(1) * quat_start_inv(3) + qs_log_qs_inv_qe(3) * quat_start_inv(1) + qs_log_qs_inv_qe(2) * quat_start_inv(0) - qs_log_qs_inv_qe(0) * quat_start_inv(2);
    qs_log_qs_inv_qe_qsinv(2) =
        qs_log_qs_inv_qe(2) * quat_start_inv(3) + qs_log_qs_inv_qe(3) * quat_start_inv(2) + qs_log_qs_inv_qe(0) * quat_start_inv(1) - qs_log_qs_inv_qe(1) * quat_start_inv(0);
    qs_log_qs_inv_qe_qsinv(3) =
        qs_log_qs_inv_qe(3) * quat_start_inv(3) - qs_log_qs_inv_qe(0) * quat_start_inv(0) - qs_log_qs_inv_qe(1) * quat_start_inv(1) - qs_log_qs_inv_qe(2) * quat_start_inv(2);

    qs_log_qs_inv_qe_qsinv = 2 * qs_log_qs_inv_qe_qsinv;

    quatS = sqrt(pow(qs_log_qs_inv_qe_qsinv(0), 2) + pow(qs_log_qs_inv_qe_qsinv(1), 2) + pow(qs_log_qs_inv_qe_qsinv(2), 2) + pow(qs_log_qs_inv_qe_qsinv(3), 2));
    // std::norm(qs_log_qs_inv_qe_qsinv);
    // std::cout << "quatS:" << quatS << std::endl;
    return quatS;
}

Quaternion ArmMathFun::QuatInterpolate(double Y, double S, Quaternion qs, Quaternion qe)
{
    Quaternion qs_inv, q0inv_q1, qs_inv_qe_t, quat;
    double costmp, tmp_;
    if (S > 0.0000000001) {
        costmp = (qs.x * qe.x) + (qs.y * qe.y) + (qs.z * qe.z) + (qs.w * qe.w);
        if (costmp < 0) {
            qe.x = -qe.x;
            qe.y = -qe.y;
            qe.z = -qe.z;
            qe.w = -qe.w;
        }
        qs_inv.x = -qs.x;
        qs_inv.y = -qs.y;
        qs_inv.z = -qs.z;
        qs_inv.w = qs.w;

        q0inv_q1.x = qs_inv.x * qe.w + qs_inv.w * qe.x + qs_inv.y * qe.z - qs_inv.z * qe.y;
        q0inv_q1.y = qs_inv.y * qe.w + qs_inv.w * qe.y + qs_inv.z * qe.x - qs_inv.x * qe.z;
        q0inv_q1.z = qs_inv.z * qe.w + qs_inv.w * qe.z + qs_inv.x * qe.y - qs_inv.y * qe.x;
        q0inv_q1.w = qs_inv.w * qe.w - qs_inv.x * qe.x - qs_inv.y * qe.y - qs_inv.z * qe.z;

        tmp_ = pow(q0inv_q1.w, 2);
        if (tmp_ > 1) {
            tmp_ = 1;
        }

        if (sqrt(1 - tmp_) < 0.0000000001) {
            qs_inv_qe_t.x = 0;
            qs_inv_qe_t.y = 0;
            qs_inv_qe_t.z = 0;
            qs_inv_qe_t.w = 1;
        } else {
            qs_inv_qe_t.x = q0inv_q1.x * sin(acos(q0inv_q1.w) * (Y / S)) / sqrt(1 - tmp_);
            qs_inv_qe_t.y = q0inv_q1.y * sin(acos(q0inv_q1.w) * (Y / S)) / sqrt(1 - tmp_);
            qs_inv_qe_t.z = q0inv_q1.z * sin(acos(q0inv_q1.w) * (Y / S)) / sqrt(1 - tmp_);
            qs_inv_qe_t.w = cos(acos(q0inv_q1.w) * (Y / S));
        }

        quat.x = qs.x * qs_inv_qe_t.w + qs.w * qs_inv_qe_t.x + qs.y * qs_inv_qe_t.z - qs.z * qs_inv_qe_t.y;
        quat.y = qs.y * qs_inv_qe_t.w + qs.w * qs_inv_qe_t.y + qs.z * qs_inv_qe_t.x - qs.x * qs_inv_qe_t.z;
        quat.z = qs.z * qs_inv_qe_t.w + qs.w * qs_inv_qe_t.z + qs.x * qs_inv_qe_t.y - qs.y * qs_inv_qe_t.x;
        quat.w = qs.w * qs_inv_qe_t.w - qs.x * qs_inv_qe_t.x - qs.y * qs_inv_qe_t.y - qs.z * qs_inv_qe_t.z;
    } else {
        quat = qs;
    }
    return quat;
}

/**
 * @description: 五次多项式插值
 * @param {double} planT离散插值总时间
 * @param {double} t离散时刻
 * @return {double}值
 */
PlanResults ArmMathFun::Plan5(double S, double planT, double t)
{
    double V_start, V_end, acc_start, acc_end, a4, a5, a6, state = 0;
    PlanResults plan;
    V_start = 0;
    V_end = 0;
    acc_start = 0;
    acc_end = 0;
    double a1 = 0;
    double a2 = V_start;
    double a3 = 0.5 * acc_start;
    a4 = (20 * S - (8 * V_end + 12 * V_start) * planT - (3 * acc_start - acc_end) * std::pow(planT, 2)) / (2 * std::pow(planT, 3));
    a5 = (-30 * S + (14 * V_end + 16 * V_start) * planT + (3 * acc_start - 2 * acc_end) * std::pow(planT, 2)) / (2 * std::pow(planT, 4));
    a6 = (12 * S - 6 * (V_end + V_start) * planT + (acc_end - acc_start) * std::pow(planT, 2)) / (2 * std::pow(planT, 5));
    state = 1;

    if (state == 1) {
        if (t <= planT) {
            plan.Y = a1 + a2 * t + a3 * std::pow(t, 2) + a4 * std::pow(t, 3) + a5 * std::pow(t, 4) + a6 * std::pow(t, 5);
            plan.A = a2 + 2 * a3 * t + 3 * a4 * std::pow(t, 2) + 4 * a5 * std::pow(t, 3) + 5 * a6 * std::pow(t, 4);
            plan.J = 2 * a3 + 6 * a4 * t + 12 * a5 * std::pow(t, 2) + 20 * a6 * std::pow(t, 3);
        } else {
            plan.Y = a1 + a2 * planT + a3 * std::pow(planT, 2) + a4 * std::pow(planT, 3) + a5 * std::pow(planT, 4) + a6 * std::pow(planT, 5);
            plan.A = a2 + 2 * a3 * planT + 3 * a4 * std::pow(planT, 2) + 4 * a5 * std::pow(planT, 3) + 5 * a6 * std::pow(planT, 4);
            plan.J = 2 * a3 + 6 * a4 * planT + 12 * a5 * std::pow(planT, 2) + 20 * a6 * std::pow(planT, 3);
        }
    } else {
        plan.Y = 0;
        state = 0;
    }
    return plan;
}

/**
 * @brief 关节角检查
 *
 * @param q 需要检查的角度
 * @param qmin 下限
 * @param qmax 上限
 * @return true
 * @return false
 */
bool ArmMathFun::CheckAngle(vector<double> q, vector<double> qmin, vector<double> qmax)
{
    bool pushState;
    if (q[0] < qmin[0] || q[0] > qmax[0] || q[1] < qmin[1] || q[1] > qmax[1] || q[2] < qmin[2] || q[2] > qmax[2] || q[3] < qmin[3] || q[3] > qmax[3] || q[4] < qmin[4] ||
        q[4] > qmax[4] || q[5] < qmin[5] || q[5] > qmax[5]) {
        pushState = false;
    } else {
        pushState = true;
    }

    // if (q(0) < qmin(0) || q(0) > qmax(0) || q(1) < qmin(1) || q(1) > qmax(1) || q(2) < qmin(2) || q(2) > qmax(2) || q(3) < qmin(3) || q(3) > qmax(3) || q(4) < qmin(4) ||
    //     q(4) > qmax(4) || q(5) < qmin(5) || q(5) > qmax(5)) {
    //     pushState = false;
    // } else {
    // pushState = true;
    // }

    return pushState;
}
Result<Vec6<double>> ArmMathFun::CheckSpeed(double UserV, Vec6<double> ParamVmax)
{
    Result<Vec6<double>> Speed;
    // Vec6<double> ParamVmax = GetArmParam().axis.V_max;
    Vec6<double> UserSpeed;
    for (int i = 0; i < 6; i++) {
        if (fabs(UserV) <= ParamVmax(i)) {
            Speed.first = RetState::ok;
            UserSpeed(i) = fabs(UserV);
        } else {
            Speed.first = RetState::outRange;
            UserSpeed(i) = 0;
        }
    }
    Speed.second = UserSpeed;
    return Speed;
}

Result<Vec6<double>> ArmMathFun::CheckAcc(double UserA, Vec6<double> ParamAmax, MoveFlag flag)
{
    Result<Vec6<double>> Acc;
    // Vec6<double> ParamVmax = GetArmParam().axis.V_max;
    Vec6<double> UserSpeed;
    if (flag == MoveFlag::movej) {
        for (int i = 0; i < 6; i++) {
            if (fabs(UserA) <= ParamAmax(i)) {
                Acc.first = RetState::ok;
                UserSpeed(i) = fabs(UserA);
            } else {
                Acc.first = RetState::outRange;
                UserSpeed(i) = 0;
            }
        }
        Acc.second = UserSpeed;
    } else if (flag == MoveFlag::movel) {
        for (int i = 0; i < 3; i++) {
            if (fabs(UserA) <= ParamAmax(i)) {
                Acc.first = RetState::ok;
                UserSpeed(i) = fabs(UserA);
                UserSpeed(i + 3) = ParamAmax(i + 3);
            } else {
                Acc.first = RetState::outRange;
                UserSpeed(i) = 0;
            }
        }
        Acc.second = UserSpeed;
    }
    return Acc;
}

RetState ArmMathFun::Checkrapid(double set)
{
    RetState ret;
    if (set >= 0 && set <= 1) {
        ret = RetState::ok;
    } else {
        ret = RetState::outRange;
    }
    return ret;
}

/**
 * @brief 修正DH的连杆变换通式
 */
Mat4<double> ArmMathFun::Ti_modified(double theta, double d, double a, double alpha)
{
    Mat4<double> Ti;
    double T11, T12, T13, T14, T21, T22, T23, T24, T31, T32, T33, T34, T41, T42, T43, T44;
    T11 = cos(theta);
    T12 = -sin(theta);
    T13 = 0;
    T14 = a;
    T21 = sin(theta) * cos(alpha);
    T22 = cos(theta) * cos(alpha);
    T23 = -sin(alpha);
    T24 = -d * sin(alpha);
    T31 = sin(theta) * sin(alpha);
    T32 = cos(theta) * sin(alpha);
    T33 = cos(alpha);
    T34 = -d * cos(alpha);
    T41 = 0;
    T42 = 0;
    T43 = 0;
    T44 = 1;
    Ti << T11, T12, T13, T14, T21, T22, T23, T24, T31, T32, T33, T34, T41, T42, T43, T44;
    return Ti;
}

/**
 * @brief 标准DH的连杆变换通式
 */
Mat4<double> ArmMathFun::Ti_standard(double theta, double d, double a, double alpha)
{
    Mat4<double> Ti;
    double T11, T12, T13, T14, T21, T22, T23, T24, T31, T32, T33, T34, T41, T42, T43, T44;
    T11 = cos(theta);
    T12 = -sin(theta) * cos(alpha);
    T13 = sin(theta) * sin(alpha);
    T14 = a * cos(theta);
    T21 = sin(theta);
    T22 = cos(theta) * cos(alpha);
    T23 = -cos(theta) * sin(alpha);
    T24 = a * sin(theta);
    T31 = 0;
    T32 = sin(alpha);
    T33 = cos(alpha);
    T34 = d;
    T41 = 0;
    T42 = 0;
    T43 = 0;
    T44 = 1;
    Ti << T11, T12, T13, T14, T21, T22, T23, T24, T31, T32, T33, T34, T41, T42, T43, T44;

    return Ti;
}
Vec6<double> ArmMathFun::Ji(Mat4<double> T)
{
    Vec6<double> Ji;
    double Ji_1, Ji_2, Ji_3, Ji_4, Ji_5, Ji_6;
    Ji_1 = -T(0, 0) * T(1, 3) + T(1, 0) * T(0, 3);
    Ji_2 = -T(0, 1) * T(1, 3) + T(1, 1) * T(0, 3);
    Ji_3 = -T(0, 2) * T(1, 3) + T(1, 2) * T(0, 3);
    Ji_4 = T(2, 0);
    Ji_5 = T(2, 1);
    Ji_6 = T(2, 2);
    Ji << Ji_1, Ji_2, Ji_3, Ji_4, Ji_5, Ji_6;
    return Ji;
}
/**
 * @brief 六轴机械臂末端坐标系下的雅克比矩阵，已知关节，求末端
 */
Mat6<double> ArmMathFun::Jacobian_tool6(vector<double> q)
{
    vector<Vec6<double>> J;
    Mat4<double> T;
    Mat6<double> Jtool;
    auto& armParam = GetArmParam();
    vector<Mat4<double>> T_list;
    for (size_t i = 0; i < q.size(); i++) {
        T_list.push_back(Ti_modified(q[i] + armParam.kine_param.offset[i], armParam.kine_param.d[i], armParam.kine_param.a[i], armParam.kine_param.alpha[i]));
    }
    T = Cartesian2RotMat(armParam.tool.tool);
    for (int i = q.size() - 1; i >= 0; i--) {
        J.push_back(Ji(T));
        T = T_list[i] * T;
    }
    reverse(J.begin(), J.end());
    for (size_t i = 0; i < J.size(); i++) {
        Jtool.col(i) = J[i];
    }

    return Jtool;
}
/**
 * @brief 六轴机械臂基座标系坐标系下的雅克比矩阵，已知末端，求关节
 */
Mat6<double> ArmMathFun::Jacobian_base6(vector<double> q)
{
    vector<Vec6<double>> J;
    Mat4<double> T, FK, Tbase, Ttool;
    Mat3<double> R;
    Mat6<double> Jtool, RR, Jbase;
    vector<Mat4<double>> T_list;
    auto& armParam = GetArmParam();

    Tbase = Cartesian2RotMat(armParam.base.base);
    Ttool = Cartesian2RotMat(armParam.tool.tool);

    for (size_t i = 0; i < q.size(); i++) {
        T_list.push_back(Ti_modified(q[i] + armParam.kine_param.offset[i], armParam.kine_param.d[i], armParam.kine_param.a[i], armParam.kine_param.alpha[i]));
    }
    T = Cartesian2RotMat(armParam.tool.tool);
    for (int i = q.size() - 1; i >= 0; i--) {
        J.push_back(Ji(T));
        T = T_list[i] * T;
    }
    reverse(J.begin(), J.end());
    for (size_t i = 0; i < J.size(); i++) {
        Jtool.col(i) = J[i];
    }
    FK = Tbase * T;
    R = FK.block(0, 0, 3, 3);
    RR.fill(0);
    RR.block(0, 0, 3, 3) = R;
    RR.block(3, 3, 6, 6) = R;
    Jbase = RR * Jtool;

    return Jbase;
}

/*
 *@brief 七自由度机器人正运动学解算
 */
pair<Pose, double> ArmMathFun::Fkine_seven(Vec7<double> Radian, Pose ToolValue)
{
    Mat4<double> Tbase, Ttool, T1, T2, T3, T4, T5, T6, T7, TT, T02, T04, T07, Usertool;
    Vec3<double> PSE0, PSW0, uSW0, LBS0, LWT7, V, P, P07, P04, P02;
    Mat3<double> I;
    Pose EndPose;
    double Phi;
    pair<Pose, double> OUT;
    auto& armParam = GetArmParam();
    Tbase = Cartesian2RotMat(armParam.base.base);
    Ttool = Cartesian2RotMat(armParam.tool.tool);
    Usertool = Cartesian2RotMat(ToolValue);

    LBS0 << 0, 0, armParam.kine_param.d[0];
    LWT7 << 0, 0, armParam.kine_param.d[6];
    I << 1, 0, 0, 0, 1, 0, 0, 0, 1;
    V << 0, 0, 1;

    T1 = Ti_standard(Radian(0), armParam.kine_param.d[0], armParam.kine_param.a[0], armParam.kine_param.alpha[0]);
    T2 = Ti_standard(Radian(1), armParam.kine_param.d[1], armParam.kine_param.a[1], armParam.kine_param.alpha[1]);
    T3 = Ti_standard(Radian(2), armParam.kine_param.d[2], armParam.kine_param.a[2], armParam.kine_param.alpha[2]);
    T4 = Ti_standard(Radian(3), armParam.kine_param.d[3], armParam.kine_param.a[3], armParam.kine_param.alpha[3]);
    T5 = Ti_standard(Radian(4), armParam.kine_param.d[4], armParam.kine_param.a[4], armParam.kine_param.alpha[4]);
    T6 = Ti_standard(Radian(5), armParam.kine_param.d[5], armParam.kine_param.a[5], armParam.kine_param.alpha[5]);
    T7 = Ti_standard(Radian(6), armParam.kine_param.d[6], armParam.kine_param.a[6], armParam.kine_param.alpha[6]);

    TT = Tbase * T1 * T2 * T3 * T4 * T5 * T6 * T7 * Ttool * Usertool;
    EndPose = RotMat2Cartesian(TT);

    T02 = Tbase * T1 * T2;
    T04 = T02 * T3 * T4;
    T07 = T04 * T5 * T6 * T7 * Ttool * Usertool;
    P02 << T02(0, 3), T02(1, 3), T02(2, 3);
    P04 << T04(0, 3), T04(1, 3), T04(2, 3);
    P07 << T07(0, 3), T07(1, 3), T07(2, 3);
    PSE0 = P04 - P02;
    PSW0 = P07 - LBS0 - T07.block(0, 0, 3, 3) * LWT7;
    uSW0 = PSW0 / PSW0.norm();
    P = (I - uSW0 * uSW0.transpose()) * PSE0;
    Phi = atan2(uSW0.transpose() * V.cross(P), V.transpose() * P);

    OUT = make_pair(EndPose, Phi);
    return OUT;
}

pair<bool, Vec7<double>> ArmMathFun::IKine_seven(Pose KPS6, double phi, Vec7<double> qRev)
{
    Vec3<double> LBS0, LSE3, LEW4, LWT7, PSW0, uSW0, PSV0, el, PSE0, y300, z300, x300, PFK;
    Mat3<double> I, R300, AS, BS, CS, AW, BW, CW, skew_el, skew_u_SW0;
    Mat4<double> D;
    Vec2<double> theta1, theta2, theta3, theta4, theta5, theta6, theta7;
    Vec7<double> Qmin, Qmax, new_q, OUT;
    Vec8<double> bz, Min;
    double ESW, Min_, zyj;
    pair<bool, Vec7<double>> IK;

    auto& armParam = GetArmParam();

    for (size_t i = 0; i < armParam.axis.S_max_real.size(); i++) {
        Qmax(i) = armParam.axis.S_max_real[i];
        Qmin(i) = armParam.axis.S_min_real[i];
    }

    Mat4<double> FK = ArmMathFun::Cartesian2RotMat(KPS6);

    I << 1, 0, 0, 0, 1, 0, 0, 0, 1;
    LBS0 << 0, 0, armParam.kine_param.d[0];
    LSE3 << 0, -armParam.kine_param.d[2], 0;
    LEW4 << 0, 0, armParam.kine_param.d[4];
    LWT7 << 0, 0, armParam.kine_param.d[6];
    PFK << FK(0, 3), FK(1, 3), FK(2, 3);
    PSW0 = PFK - LBS0 - FK.block(0, 0, 3, 3) * LWT7;

    theta4(0) = M_PI - acos((pow(LSE3.norm(), 2) + pow(LEW4.norm(), 2) - pow(PSW0.norm(), 2)) / (2 * LSE3.norm() * LEW4.norm()));

    theta4(1) = -theta4(0);

    ESW = acos((pow(LSE3.norm(), 2) + pow(PSW0.norm(), 2) - pow(LEW4.norm(), 2)) / (2 * LSE3.norm() * PSW0.norm()));
    uSW0 = PSW0 / PSW0.norm();
    PSV0 << PSW0(0, 0), PSW0(1, 0), 0;
    el = PSV0.cross(PSW0) / (PSV0.cross(PSW0)).norm();
    skew_el << 0, -el(2, 0), el(1, 0), el(2, 0), 0, -el(0, 0), -el(1, 0), el(0, 0), 0;
    skew_u_SW0 << 0, -uSW0(2, 0), uSW0(1, 0), uSW0(2, 0), 0, -uSW0(0, 0), -uSW0(1, 0), uSW0(0, 0), 0;
    PSE0 = LSE3.norm() * (I + skew_el * sin(ESW) + skew_el * skew_el * (1 - cos(ESW))) * PSW0 / PSW0.norm();
    y300 = -PSE0 / PSE0.norm();
    z300 = -sign(sin(theta4(0))) * el;
    x300 = y300.cross(z300);
    R300 << x300, y300, z300;
    AS = skew_u_SW0 * R300;
    BS = -skew_u_SW0 * skew_u_SW0 * R300;
    CS = (I + skew_u_SW0 * skew_u_SW0) * R300;

    theta1(0) = atan2(-(AS(1, 1) * sin(phi) + BS(1, 1) * cos(phi) + CS(1, 1)), -(AS(0, 1) * sin(phi) + BS(0, 1) * cos(phi) + CS(0, 1)));
    theta1(1) = atan2(AS(1, 1) * sin(phi) + BS(1, 1) * cos(phi) + CS(1, 1), AS(0, 1) * sin(phi) + BS(0, 1) * cos(phi) + CS(0, 1));
    theta2(0) = acos(-AS(2, 1) * sin(phi) - BS(2, 1) * cos(phi) - CS(2, 1));
    theta2(1) = -theta2(0);
    theta3(0) = atan2((AS(2, 2) * sin(phi) + BS(2, 2) * cos(phi) + CS(2, 2)), (-AS(2, 0) * sin(phi) - BS(2, 0) * cos(phi) - CS(2, 0)));
    theta3(1) = atan2(-(AS(2, 2) * sin(phi) + BS(2, 2) * cos(phi) + CS(2, 2)), AS(2, 0) * sin(phi) + BS(2, 0) * cos(phi) + CS(2, 0));
    D << cos(theta4(0)), 0, sin(theta4(0)), 0, sin(theta4(0)), 0, -cos(theta4(0)), 0, 0, 1, 0, 0, 0, 0, 0, 1;

    AW = ((D.block<3, 3>(0, 0)).transpose()) * (AS.transpose()) * (FK.block(0, 0, 3, 3));
    BW = ((D.block<3, 3>(0, 0)).transpose()) * (BS.transpose()) * (FK.block(0, 0, 3, 3));
    CW = ((D.block<3, 3>(0, 0)).transpose()) * (CS.transpose()) * (FK.block(0, 0, 3, 3));
    theta5(0) = atan2((AW(1, 2) * sin(phi) + BW(1, 2) * cos(phi) + CW(1, 2)), (AW(0, 2) * sin(phi) + BW(0, 2) * cos(phi) + CW(0, 2)));
    theta5(1) = atan2(-(AW(1, 2) * sin(phi) + BW(1, 2) * cos(phi) + CW(1, 2)), -(AW(0, 2) * sin(phi) + BW(0, 2) * cos(phi) + CW(0, 2)));
    theta6(0) = acos(AW(2, 2) * sin(phi) + BW(2, 2) * cos(phi) + CW(2, 2));
    theta6(1) = -theta6(0);
    theta7(0) = atan2((AW(2, 1) * sin(phi) + BW(2, 1) * cos(phi) + CW(2, 1)), (-AW(2, 0) * sin(phi) - BW(2, 0) * cos(phi) - CW(2, 0)));
    theta7(1) = atan2(-(AW(2, 1) * sin(phi) + BW(2, 1) * cos(phi) + CW(2, 1)), (AW(2, 0) * sin(phi) + BW(2, 0) * cos(phi) + CW(2, 0)));
    Mat87<double> QQ;
    // 拼成矩阵，做选解
    QQ << theta1(0), theta2(0), theta3(0), theta4(0), theta5(0), theta6(0), theta7(0), theta1(0), theta2(0), theta3(0), theta4(0), theta5(1), theta6(1), theta7(1), theta1(0),
        theta2(0), theta3(1), theta4(1), theta5(1), theta6(0), theta7(0), theta1(0), theta2(0), theta3(1), theta4(1), theta5(0), theta6(1), theta7(1), theta1(1), theta2(1),
        theta3(1), theta4(0), theta5(0), theta6(0), theta7(0), theta1(1), theta2(1), theta3(1), theta4(0), theta5(1), theta6(1), theta7(1), theta1(1), theta2(1), theta3(0),
        theta4(1), theta5(1), theta6(0), theta7(0), theta1(1), theta2(1), theta3(0), theta4(1), theta5(0), theta6(1), theta7(1);

    // cout << "QQ:" << QQ << endl;
    // cout << "Qmax:" << Qmax << endl;
    // cout << "Qmin:" << Qmin << endl;

    //      选取最优解，从六轴部分复制
    for (int i = 0; i < 8; i++) {
        if ((QQ(i, 0) >= Qmin(0)) && (QQ(i, 0) <= Qmax(0)) && (QQ(i, 1) >= Qmin(1)) && (QQ(i, 1) <= Qmax(1)) && (QQ(i, 2) >= Qmin(2)) && (QQ(i, 2) <= Qmax(2)) &&
            (QQ(i, 3) >= Qmin(3)) && (QQ(i, 3) <= Qmax(3)) && (QQ(i, 4) >= Qmin(4)) && (QQ(i, 4) <= Qmax(4)) && (QQ(i, 5) >= Qmin(5)) && (QQ(i, 5) <= Qmax(5)) &&
            (QQ(i, 6) >= Qmin(6)) && (QQ(i, 6) <= Qmax(6))) {
            bz(i) = 1;
            Min(i) = fabs(qRev(0) - QQ(i, 0)) + fabs(qRev(1) - QQ(i, 1)) + fabs(qRev(2) - QQ(i, 2)) + fabs(qRev(3) - QQ(i, 3)) + fabs(qRev(4) - QQ(i, 4)) +
                     fabs(qRev(5) - QQ(i, 5)) + fabs(qRev(6) - QQ(i, 6));
        } else {
            bz(i) = 0;
            Min(i) = 0;
        }
    }

    zyj = 8;
    Min_ = Min(0) + Min(1) + Min(2) + Min(3) + Min(4) + Min(5) + Min(6) + Min(7);
    for (int i = 0; i < 8; i++) {
        if (bz(i) == 1 && Min_ >= Min(i)) {
            Min_ = Min(i);
            zyj = i;
        }
    }

    if (zyj == 8) {
        IK.first = false;
        for (int i = 0; i < 7; i++) {
            new_q(i) = qRev(i);
        }
    } else {  // best
        IK.first = true;
        for (int i = 0; i < 7; i++) {
            new_q(i) = QQ(zyj, i);
        }
    }
    IK.second = new_q;
    return IK;
}

int ArmMathFun::sign(double x) { return (x > 0) - (x < 0); }

bool ArmMathFun::CheckAngle_seven(vector<double> q, vector<double> qmin, vector<double> qmax)
{
    bool pushState;

    if (q[0] < qmin[0] || q[0] > qmax[0] || q[1] < qmin[1] || q[1] > qmax[1] || q[2] < qmin[2] || q[2] > qmax[2] || q[3] < qmin[3] || q[3] > qmax[3] || q[4] < qmin[4] ||
        q[4] > qmax[4] || q[5] < qmin[5] || q[5] > qmax[5] || q[6] < qmin[6] || q[6] > qmax[6]) {
        pushState = false;
    } else {
        pushState = true;
    }

    return pushState;
}
Result<Vec7<double>> ArmMathFun::CheckSpeed_seven(double UserV, Vec7<double> ParamVmax)
{
    Result<Vec7<double>> Speed;
    // Vec6<double> ParamVmax = GetArmParam().axis.V_max;
    Vec7<double> UserSpeed;
    for (int i = 0; i < 7; i++) {
        if (fabs(UserV) <= ParamVmax[i]) {
            Speed.first = RetState::ok;
            UserSpeed(i) = fabs(UserV);
        } else {
            Speed.first = RetState::outRange;
            UserSpeed.fill(0);
        }
    }
    Speed.second = UserSpeed;
    return Speed;
}

Result<Vec7<double>> ArmMathFun::CheckAcc_seven(double UserA, Vec7<double> ParamAmax, MoveFlag flag)
{
    Result<Vec7<double>> Acc;
    // Vec6<double> ParamVmax = GetArmParam().axis.V_max;
    Vec7<double> UserAcc;
    if (flag == MoveFlag::movej) {
        for (int i = 0; i < 7; i++) {
            if (fabs(UserA) <= ParamAmax(i)) {
                Acc.first = RetState::ok;
                UserAcc(i) = fabs(UserA);
            } else {
                Acc.first = RetState::outRange;
                UserAcc(i) = 0;
            }
        }
        Acc.second = UserAcc;
    } else if (flag == MoveFlag::movel) {
        for (int i = 0; i < 3; i++) {
            if (fabs(UserA) <= ParamAmax(i)) {
                Acc.first = RetState::ok;
                UserAcc(i) = fabs(UserA);
                UserAcc(i + 3) = ParamAmax(i + 3);
            } else {
                Acc.first = RetState::outRange;
                UserAcc(i) = 0;
            }
        }
        UserAcc(6) = ParamAmax(6);
        Acc.second = UserAcc;
    }
    return Acc;
}

/**
 * @brief 配置文件中的转动惯量信息转为对角矩阵
 *
 * @param input 顺序为Icxx, Icyy, Iczz, Icxy, Icxz, Icyz
 * @return Mat3<double> 对角矩阵
 */
Mat3<double> ArmMathFun::Vec2DiagMat(Vec6<double> input)
{
    //
    Mat3<double> outPut;
    outPut << input(0), input(3), input(4), input(3), input(1), input(5), input(4), input(5), input(2);
    return outPut;
}
/**
 * @brief 将齐次矩阵转换为旋转矩阵和平移向量
 *
 * @param input 齐次矩阵
 * @return std::pair<Mat3<double>, Vec3<double>>
 */
std::pair<Mat3<double>, Vec3<double>> ArmMathFun::RotMat2RP(Mat4<double> input)
{
    pair<Mat3<double>, Vec3<double>> output;
    Mat3<double> R;
    Vec3<double> P;
    R = input.block(0, 0, 3, 3);
    P = input.block(0, 3, 3, 1);
    return output;
}
/**
 * @brief 计算力矩
 *
 * @param q
 * @param qdot
 * @param qddot
 * @return Vec6<double>
 */
Vec6<double> ArmMathFun::Dynamic(Vec6<double> q, Vec6<double> qdot, Vec6<double> qddot)
{
    Vec6<double> out;
    (void)q;
    (void)qdot;
    (void)qddot;

    return out;
}
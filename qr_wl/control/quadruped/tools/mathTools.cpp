

#include "mathTools.hpp"

#include <iostream>

#include "baseline.hpp"
#include "quadrupedParam.hpp"

using namespace ::std;
/**
 * @description:atan2的结果(-pi~pi)改变值域
 * @param {double} value atan2的值域(-pi~pi)
 * @return {double} Roll值域(0~2Pi)
 */
double MathFun::atan2toRoll(double value)
{
    if (value < 0)
    {
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
Vec3<double> MathFun::Rz(const Vec3<double> &pIn, double radZ)
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
Vec3<double> MathFun::Ry(const Vec3<double> &pIn, double radY)
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
Vec3<double> MathFun::Rx(const Vec3<double> &pIn, double radX)
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
Vec3<double> MathFun::quaternion2Euler(const Vec4<double> &q)
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
        euler(1) = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        euler(1) = asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q(0) * q(3) + q(1) * q(2));
    double cosy_cosp = 1 - 2 * (q(2) * q(2) + q(3) * q(3));
    euler(2) = atan2(siny_cosp, cosy_cosp);

    return euler;
}

Vec3<double> MathFun::BezierCurve3(const Vec3<double> &p0, const Vec3<double> &p1, const Vec3<double> &pA, double s, double maxS)
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
Vec3<double> MathFun::BezierCurve4(const Vec3<double> &p0, const Vec3<double> &p1, const Vec3<double> &pA, const Vec3<double> &pB, double s, double maxS)
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
Vec3<double> MathFun::BezierCurve5(const Vec3<double> &p0, const Vec3<double> &p1, const Vec3<double> &pA, const Vec3<double> &pB, const Vec3<double> &pC, double s, double maxS)
{
    Vec3<double> pi;
    pi.setZero();
    s = (1.0 / maxS) * s;
    pi = p0 * pow((1 - s), 4) + 4 * pA * s * pow((1 - s), 3) + 6 * pB * pow(s, 2) * pow((1 - s), 2) + 4 * pC * (1 - s) * pow(s, 3) + p1 * pow(s, 4);
    // cout << "s = " << s << endl;
    return pi;
}
Vec3<double> MathFun::BezierCurve7(const Vec3<double> &p0, const Vec3<double> &p1, const Vec3<double> &pA, const Vec3<double> &pB, const Vec3<double> &pC, const Vec3<double> &pD,
                                   const Vec3<double> &pE, double s, double maxS)
{
    Vec3<double> pi;
    pi.setZero();
    s = (1.0 / maxS) * s;
    pi = p0 * pow((1 - s), 6) + 6 * pA * s * pow((1 - s), 5) + 15 * pB * pow(s, 2) * pow((1 - s), 4) + 20 * pC * pow((1 - s), 3) * pow(s, 3) + 15 * pD * pow(1 - s, 2) * pow(s, 4) +
         6 * pE * (1 - s) * pow(s, 5) + p1 * pow(s, 6);
    // cout << "s = " << s << endl;
    return pi;
}

Vec3<double> MathFun::CycloidCurve(const Vec3<double> &p0, const Vec3<double> &p1, double h, double t, double T)
{
    // p取值范围为0-1
    double p = t / T;
    Vec3<double> pt;
    double phi = 2 * M_PI * p - sin(2 * M_PI * p);
    double theta = 1 - cos(2 * M_PI * p);
    pt(0) = p0(0) + (p1(0) - p0(0)) * phi / (2 * M_PI);
    pt(1) = p0(1) + (p1(1) - p0(1)) * phi / (2 * M_PI);
    pt(2) = p0(2) + (h / 2) * theta; // 宇树的书124页表达式，这样会导致腿越走越长或越走越短,20230324做了修改起始点和圆半径，轨迹不佳，暂时弃用
    return pt;
}
/**
 * @description:根据一阶贝塞尔曲线生成轨迹点
 * @param {Vec3<double> p0, Vec3<double>} p1起点，终点
 * @param {double} s当前离散时刻
 * @param {double} maxS总离散时间
 * @return {Vec3<double>}贝塞尔曲线上的s时刻的坐标
 */
Vec3<double> MathFun::BezierCurve2(const Vec3<double> &p0, const Vec3<double> &p1, double s, double maxS)
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
double MathFun::deg2rad(double deg) { return (M_PI * deg / 180); }
/**
 * @description: 弧度转角度
 * @param {double} rad弧度
 * @return {double}角度
 */
double MathFun::rad2deg(double rad) { return (180 * rad / M_PI); }
/**
 * @description: 改变atan2值域
 * @param {double} atan2Value 值域(-pi~pi)
 * @return {double} 值域(0~2pi)
 */
double MathFun::changeAtan2Range(double atan2Value)
{
    if (atan2Value >= 0)
    {
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
double MathFun::Poly5(double T, double t)
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
double MathFun::Sign(double value)
{
    if (value >= 0)
    {
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
double MathFun::setValueRange(double value, double rangeA, double rangeB, bool isPrint, int ErrIndex)
{
    double result = 0.0;
    if (rangeB >= rangeA)
    {
        if (value >= rangeB)
        {
            result = rangeB;
            if (isPrint)
            {
                cout << "out of rangB: " << rangeB << " ErrIndex:" << ErrIndex << endl;
            }
        }
        else if (value <= rangeA)
        {
            result = rangeA;
            if (isPrint)
            {
                cout << "out of rangA: " << rangeA << " ErrIndex:" << ErrIndex << endl;
            }
        }
        else
        {
            result = value;
        }
    }
    else
    {
        if (isPrint)
        {
            cout << "rangeA > rangeB. ErrIndex :" << ErrIndex << endl;
        }
    }
    return result;
}

double MathFun::SetValueRange(double value, double min, double max, const std::string &str, bool isPrint)
{
    double result = 0.0;
    if (max >= min)
    {
        if (value > max)
        {
            result = max;
            if (setValueRangePrint_.count(str) == 0)
            {
                setValueRangePrint_.insert(str);
                if (isPrint)
                {
                    LOG_WARN("{} > MAX! max:{}, value:{}", str, max, value);
                }
            }
        }
        else if (value < min)
        {
            result = min;
            if (setValueRangePrint_.count(str) == 0)
            {
                setValueRangePrint_.insert(str);
                if (isPrint)
                {
                    LOG_WARN("{} < MIN! min:{}, value:{}", str, min, value);
                }
            }
        }
        else
        {
            result = value;
            setValueRangePrint_.erase(str);
        }
    }
    else
    {
        if (isPrint)
        {
            LOG_ERROR("MIN < MAX! min:{}, max:{}", min, max);
        }
    }
    return result;
}

Mat3<double> MathFun::rotVec2rotMat(Vec3<double> rVec)
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
    if (absxk > 1.29246971E-26F)
    {
        theta = 1.0F;
        scale = absxk;
    }
    else
    {
        t = absxk / 1.29246971E-26F;
        theta = t * t;
    }

    absxk = std::fabs(rVec(1));
    if (absxk > scale)
    {
        t = scale / absxk;
        theta = theta * t * t + 1.0F;
        scale = absxk;
    }
    else
    {
        t = absxk / scale;
        theta += t * t;
    }

    absxk = std::fabs(rVec(2));
    if (absxk > scale)
    {
        t = scale / absxk;
        theta = theta * t * t + 1.0F;
        scale = absxk;
    }
    else
    {
        t = absxk / scale;
        theta += t * t;
    }

    theta = scale * std::sqrt(theta);
    if (theta < 1.0E-6)
    {
        for (i = 0; i < 9; i++)
        {
            a[i] = 0.0F;
        }

        a[0] = 1.0F;
        a[4] = 1.0F;
        a[8] = 1.0F;
    }
    else
    {
        absxk = rVec(0) / theta;
        u[0] = absxk;
        t = rVec(1) / theta;
        u[1] = t;
        u_tmp = rVec(2) / theta;
        u[2] = u_tmp;
        alpha = std::cos(theta);
        scale = std::sin(theta);
        for (i = 0; i < 9; i++)
        {
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
        for (i = 0; i < 3; i++)
        {
            rotationVector[3 * i] = absxk * u[i];
            rotationVector[3 * i + 1] = t * u[i];
            rotationVector[3 * i + 2] = u_tmp * u[i];
        }

        for (i = 0; i < 9; i++)
        {
            a[i] = (static_cast<double>(b_a[i]) * alpha + a[i]) + (1.0F - alpha) * rotationVector[i];
        }
    }

    Mat3<double> tempR;
    for (i = 0; i < 3; i++)
    {
        tempR(0, i) = a[i];
        tempR(1, i) = a[i + 3];
        tempR(2, i) = a[i + 6];
    }
    return tempR;
}

MathFun::MathFun() { rt_InitInfAndNaN(); }

void MathFun::rt_InitInfAndNaN()
{
    rtNaN = std::numeric_limits<double>::quiet_NaN();
    rtNaNF = std::numeric_limits<double>::quiet_NaN();
    rtInf = std::numeric_limits<double>::infinity();
    rtInfF = std::numeric_limits<double>::infinity();
    rtMinusInf = -std::numeric_limits<double>::infinity();
    rtMinusInfF = -std::numeric_limits<double>::infinity();
}

bool MathFun::rtIsInf(double value) { return ((value == rtInf || value == rtMinusInf) ? true : false); }

bool MathFun::rtIsInfF(double value) { return (((value) == rtInfF || (value) == rtMinusInfF) ? true : false); }

bool MathFun::rtIsNaN(double value) { return ((value != value) ? true : false); }

bool MathFun::rtIsNaNF(double value) { return ((value != value) ? true : false); }

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
    kend = (ix0 + n) - 1; // todo 容易越界
    for (k = ix0; k <= kend; k++)
    {
        absxk = std::fabs(x[k - 1]);
        if (absxk > scale)
        {
            t = scale / absxk;
            y = y * t * t + 1.0F;
            scale = absxk;
        }
        else
        {
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
    if (!(a == 0.0F))
    {
        ix = ix0 - 1;
        iy = iy0 - 1;
        i = n - 1;
        for (k = 0; k <= i; k++)
        {
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
    for (k = 0; k < n; k++)
    {
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
    for (k = ix0; k <= kend; k++)
    {
        absxk = std::fabs(x[k - 1]);
        if (absxk > scale)
        {
            t = scale / absxk;
            y = y * t * t + 1.0F;
            scale = absxk;
        }
        else
        {
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
    if (!(a == 0.0F))
    {
        ix = ix0 - 1;
        iy = iy0 - 1;
        i = n - 1;
        for (k = 0; k <= i; k++)
        {
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
    if (!(a == 0.0F))
    {
        ix = ix0 - 1;
        iy = iy0 - 1;
        i = n - 1;
        for (k = 0; k <= i; k++)
        {
            y[iy] += a * x[ix];
            ix++;
            iy++;
        }
    }
}

static void xrotg(double *a, double *b, double *c, double *s)
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
    if (absa > absb)
    {
        roe = *a;
    }

    scale = absa + absb;
    if (scale == 0.0F)
    {
        *s = 0.0F;
        *c = 1.0F;
        *a = 0.0F;
        *b = 0.0F;
    }
    else
    {
        ads = absa / scale;
        bds = absb / scale;
        scale *= std::sqrt(ads * ads + bds * bds);
        if (roe < 0.0F)
        {
            scale = -scale;
        }

        *c = *a / scale;
        *s = *b / scale;
        if (absa > absb)
        {
            *b = *s;
        }
        else if (*c != 0.0F)
        {
            *b = 1.0F / *c;
        }
        else
        {
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

void MathFun::xswap(double x[9], int ix0, int iy0)
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

void MathFun::svd(const double A[9], double U[9], double s[3], double V[9])
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
    for (kase = 0; kase < 9; kase++)
    {
        b_A[kase] = A[kase];
        U[kase] = 0.0F;
        V[kase] = 0.0F;
    }

    apply_transform = false;
    nrm = xnrm2(3, b_A, 1);
    if (nrm > 0.0F)
    {
        apply_transform = true;
        if (b_A[0] < 0.0F)
        {
            nrm = -nrm;
        }

        if (std::fabs(nrm) >= 9.86076132E-32F)
        {
            r = 1.0F / nrm;
            for (qp1 = 1; qp1 < 4; qp1++)
            {
                b_A[qp1 - 1] *= r;
            }
        }
        else
        {
            for (qp1 = 1; qp1 < 4; qp1++)
            {
                b_A[qp1 - 1] /= nrm;
            }
        }

        b_A[0]++;
        b_s[0] = -nrm;
    }
    else
    {
        b_s[0] = 0.0F;
    }

    for (kase = 2; kase < 4; kase++)
    {
        qjj = 3 * (kase - 1);
        if (apply_transform)
        {
            xaxpy(3, -(xdotc(3, b_A, 1, b_A, qjj + 1) / b_A[0]), 1, b_A, qjj + 1);
        }

        e[kase - 1] = b_A[qjj];
    }

    for (qp1 = 1; qp1 < 4; qp1++)
    {
        U[qp1 - 1] = b_A[qp1 - 1];
    }

    nrm = b_xnrm2(e, 2);
    if (nrm == 0.0F)
    {
        e[0] = 0.0F;
    }
    else
    {
        if (e[1] < 0.0F)
        {
            e[0] = -nrm;
        }
        else
        {
            e[0] = nrm;
        }

        r = e[0];
        if (std::fabs(e[0]) >= 9.86076132E-32F)
        {
            r = 1.0F / e[0];
            for (qp1 = 2; qp1 < 4; qp1++)
            {
                e[qp1 - 1] *= r;
            }
        }
        else
        {
            for (qp1 = 2; qp1 < 4; qp1++)
            {
                e[qp1 - 1] /= r;
            }
        }

        e[1]++;
        e[0] = -e[0];
        for (qp1 = 2; qp1 < 4; qp1++)
        {
            work[qp1 - 1] = 0.0F;
        }

        for (kase = 2; kase < 4; kase++)
        {
            b_xaxpy(2, e[kase - 1], b_A, 3 * (kase - 1) + 2, work, 2);
        }

        for (kase = 2; kase < 4; kase++)
        {
            c_xaxpy(2, -e[kase - 1] / e[1], work, 2, b_A, 3 * (kase - 1) + 2);
        }
    }

    for (qp1 = 2; qp1 < 4; qp1++)
    {
        V[qp1 - 1] = e[qp1 - 1];
    }

    apply_transform = false;
    nrm = xnrm2(2, b_A, 5);
    if (nrm > 0.0F)
    {
        apply_transform = true;
        if (b_A[4] < 0.0F)
        {
            nrm = -nrm;
        }

        if (std::fabs(nrm) >= 9.86076132E-32F)
        {
            r = 1.0F / nrm;
            for (qp1 = 5; qp1 < 7; qp1++)
            {
                b_A[qp1 - 1] *= r;
            }
        }
        else
        {
            for (qp1 = 5; qp1 < 7; qp1++)
            {
                b_A[qp1 - 1] /= nrm;
            }
        }

        b_A[4]++;
        b_s[1] = -nrm;
    }
    else
    {
        b_s[1] = 0.0F;
    }

    for (kase = 3; kase < 4; kase++)
    {
        if (apply_transform)
        {
            xaxpy(2, -(xdotc(2, b_A, 5, b_A, 8) / b_A[4]), 5, b_A, 8);
        }
    }

    for (qp1 = 2; qp1 < 4; qp1++)
    {
        U[qp1 + 2] = b_A[qp1 + 2];
    }

    m = 1;
    b_s[2] = b_A[8];
    e[1] = b_A[7];
    e[2] = 0.0F;
    U[6] = 0.0F;
    U[7] = 0.0F;
    U[8] = 1.0F;
    for (q = 1; q >= 0; q--)
    {
        qp1 = q + 2;
        qq = q + 3 * q;
        if (b_s[q] != 0.0F)
        {
            for (kase = qp1; kase < 4; kase++)
            {
                qjj = (q + 3 * (kase - 1)) + 1;
                xaxpy(3 - q, -(xdotc(3 - q, U, qq + 1, U, qjj) / U[qq]), qq + 1, U, qjj);
            }

            for (qp1 = q + 1; qp1 < 4; qp1++)
            {
                kase = (qp1 + 3 * q) - 1;
                U[kase] = -U[kase];
            }

            U[qq]++;
            if (0 <= q - 1)
            {
                U[3 * q] = 0.0F;
            }
        }
        else
        {
            U[3 * q] = 0.0F;
            U[3 * q + 1] = 0.0F;
            U[3 * q + 2] = 0.0F;
            U[qq] = 1.0F;
        }
    }

    for (q = 2; q >= 0; q--)
    {
        if ((q + 1 <= 1) && (e[0] != 0.0F))
        {
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
    for (q = 0; q < 3; q++)
    {
        if (b_s[q] != 0.0F)
        {
            nrm = std::fabs(b_s[q]);
            r = b_s[q] / nrm;
            b_s[q] = nrm;
            if (q + 1 < 3)
            {
                e[q] /= r;
            }

            qjj = 3 * q;
            kase = qjj + 3;
            for (qp1 = qjj + 1; qp1 <= kase; qp1++)
            {
                U[qp1 - 1] *= r;
            }
        }

        if ((q + 1 < 3) && (e[q] != 0.0F))
        {
            nrm = std::fabs(e[q]);
            r = nrm / e[q];
            e[q] = nrm;
            b_s[q + 1] *= r;
            qjj = 3 * (q + 1);
            kase = qjj + 3;
            for (qp1 = qjj + 1; qp1 <= kase; qp1++)
            {
                V[qp1 - 1] *= r;
            }
        }

        nrm = std::fabs(b_s[q]);
        r = std::fabs(e[q]);
        if ((nrm > r) || MathFun::rtIsNaNF(r))
        {
            r = nrm;
        }

        if ((!(snorm > r)) && (!MathFun::rtIsNaNF(r)))
        {
            snorm = r;
        }
    }

    while ((m + 2 > 0) && (qq < 75))
    {
        qp1 = m;
        do
        {
            exitg1 = 0;
            q = qp1 + 1;
            if (qp1 + 1 == 0)
            {
                exitg1 = 1;
            }
            else
            {
                nrm = std::fabs(e[qp1]);
                if ((nrm <= 1.1920929E-7F * (std::fabs(b_s[qp1]) + std::fabs(b_s[qp1 + 1]))) || (nrm <= 9.86076132E-32F) || ((qq > 20) && (nrm <= 1.1920929E-7F * snorm)))
                {
                    e[qp1] = 0.0F;
                    exitg1 = 1;
                }
                else
                {
                    qp1--;
                }
            }
        } while (exitg1 == 0);

        if (qp1 + 1 == m + 1)
        {
            kase = 4;
        }
        else
        {
            qjj = m + 2;
            kase = m + 2;
            exitg2 = false;
            while ((!exitg2) && (kase >= qp1 + 1))
            {
                qjj = kase;
                if (kase == qp1 + 1)
                {
                    exitg2 = true;
                }
                else
                {
                    nrm = 0.0F;
                    if (kase < m + 2)
                    {
                        nrm = std::fabs(e[kase - 1]);
                    }

                    if (kase > qp1 + 2)
                    {
                        nrm += std::fabs(e[kase - 2]);
                    }

                    r = std::fabs(b_s[kase - 1]);
                    if ((r <= 1.1920929E-7F * nrm) || (r <= 9.86076132E-32F))
                    {
                        b_s[kase - 1] = 0.0F;
                        exitg2 = true;
                    }
                    else
                    {
                        kase--;
                    }
                }
            }

            if (qjj == qp1 + 1)
            {
                kase = 3;
            }
            else if (qjj == m + 2)
            {
                kase = 1;
            }
            else
            {
                kase = 2;
                q = qjj;
            }
        }

        switch (kase)
        {
        case 1:
            r = e[m];
            e[m] = 0.0F;
            kase = m + 1;
            for (qp1 = kase; qp1 >= q + 1; qp1--)
            {
                xrotg(&b_s[qp1 - 1], &r, &sm, &sqds);
                if (qp1 > q + 1)
                {
                    r = -sqds * e[0];
                    e[0] *= sm;
                }

                xrot(V, 3 * (qp1 - 1) + 1, 3 * (m + 1) + 1, sm, sqds);
            }
            break;

        case 2:
            r = e[q - 1];
            e[q - 1] = 0.0F;
            for (qp1 = q + 1; qp1 <= m + 2; qp1++)
            {
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
            if ((!(scale > r)) && (!rtIsNaNF(r)))
            {
                scale = r;
            }

            r = std::fabs(e[m]);
            if ((!(scale > r)) && (!rtIsNaNF(r)))
            {
                scale = r;
            }

            r = std::fabs(b_s[q]);
            if ((!(scale > r)) && (!rtIsNaNF(r)))
            {
                scale = r;
            }

            r = std::fabs(e[q]);
            if ((!(scale > r)) && (!rtIsNaNF(r)))
            {
                scale = r;
            }

            sm = nrm / scale;
            nrm = b_s[m] / scale;
            r = e[m] / scale;
            sqds = b_s[q] / scale;
            b = ((nrm + sm) * (nrm - sm) + r * r) / 2.0F;
            nrm = sm * r;
            nrm *= nrm;
            if ((b != 0.0F) || (nrm != 0.0F))
            {
                r = std::sqrt(b * b + nrm);
                if (b < 0.0F)
                {
                    r = -r;
                }

                r = nrm / (b + r);
            }
            else
            {
                r = 0.0F;
            }

            r += (sqds + sm) * (sqds - sm);
            nrm = sqds * (e[q] / scale);
            for (qp1 = q + 1; qp1 <= kase; qp1++)
            {
                xrotg(&r, &nrm, &sm, &sqds);
                if (qp1 > q + 1)
                {
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
            if (b_s[q] < 0.0F)
            {
                b_s[q] = -b_s[q];
                qjj = 3 * q;
                kase = qjj + 3;
                for (qp1 = qjj + 1; qp1 <= kase; qp1++)
                {
                    V[qp1 - 1] = -V[qp1 - 1];
                }
            }

            qp1 = q + 1;
            while ((q + 1 < 3) && (b_s[q] < b_s[qp1]))
            {
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

Vec3<double> MathFun::rotMat2rotVec(Mat3<double> R)
{
    Vec3<double> rVec;
    int r;
    bool p;
    double rotationMatrix[9];
    int k;
    double U[9];
    double s[3];
    double V[9];
    double t;
    double absxk;
    double theta;
    double b_s;
    int idx;
    bool exitg1;
    int iidx;
    double y;
    // 由于构造函数mathfun时，已经初始化数值计算的边界值，奇异值，因此这语句不再需要，注意！
    /*
    if (isInitialized_rotMat2rotVec == false) {
        rotMat2rotVec_initialize();
    }
    */
    for (r = 0; r < 3; r++)
    {
        // rotationMatrix[3 * r] = R[r];
        // rotationMatrix[3 * r + 1] = R[r + 3];
        // rotationMatrix[3 * r + 2] = R[r + 6];

        rotationMatrix[3 * r] = R(r, 0);
        rotationMatrix[3 * r + 1] = R(r, 1);
        rotationMatrix[3 * r + 2] = R(r, 2);
    }

    p = true;
    for (k = 0; k < 9; k++)
    {
        if ((!p) || (rtIsInfF(rotationMatrix[k]) || rtIsNaNF(rotationMatrix[k])))
        {
            p = false;
        }
    }

    if (p)
    {
        svd(rotationMatrix, U, s, V);
    }
    else
    {
        for (r = 0; r < 9; r++)
        {
            U[r] = rtNaNF;
            V[r] = rtNaNF;
        }
    }

    for (r = 0; r < 3; r++)
    {
        absxk = U[r + 3];
        b_s = U[r + 6];
        for (idx = 0; idx < 3; idx++)
        {
            rotationMatrix[r + 3 * idx] = (U[r] * V[idx] + absxk * V[idx + 3]) + b_s * V[idx + 6];
        }
    }

    t = (rotationMatrix[0] + rotationMatrix[4]) + rotationMatrix[8];
    theta = std::acos((t - 1.0F) / 2.0F);
    rVec(0) = rotationMatrix[5] - rotationMatrix[7];
    rVec(1) = rotationMatrix[6] - rotationMatrix[2];
    rVec(2) = rotationMatrix[1] - rotationMatrix[3];
    absxk = std::sin(theta);
    if (absxk >= 0.0001F)
    {
        b_s = 1.0F / (2.0F * absxk);
        rVec(0) = theta * (rVec(0) * b_s);
        rVec(1) = theta * (rVec(1) * b_s);
        rVec(2) = theta * (rVec(2) * b_s);
    }
    else if (t - 1.0F > 0.0F)
    {
        b_s = (t - 3.0F) / 12.0F;
        rVec(0) *= 0.5F - b_s;
        rVec(1) *= 0.5F - b_s;
        rVec(2) *= 0.5F - b_s;
    }
    else
    {
        s[0] = rotationMatrix[0];
        s[1] = rotationMatrix[4];
        s[2] = rotationMatrix[8];
        if (!rtIsNaNF(rotationMatrix[0]))
        {
            idx = 1;
        }
        else
        {
            idx = 0;
            k = 2;
            exitg1 = false;
            while ((!exitg1) && (k < 4))
            {
                if (!rtIsNaNF(s[k - 1]))
                {
                    idx = k;
                    exitg1 = true;
                }
                else
                {
                    k++;
                }
            }
        }

        if (idx == 0)
        {
            iidx = 0;
        }
        else
        {
            b_s = s[idx - 1];
            iidx = idx - 1;
            r = idx + 1;
            for (k = r; k < 4; k++)
            {
                absxk = s[k - 1];
                if (b_s < absxk)
                {
                    b_s = absxk;
                    iidx = k - 1;
                }
            }
        }

        idx = iidx + 1;
        r = static_cast<int>(std::fmod(static_cast<double>(idx), 3.0));
        idx = static_cast<int>(std::fmod(static_cast<double>(idx) + 1.0, 3.0));
        b_s = std::sqrt(((rotationMatrix[iidx + 3 * iidx] - rotationMatrix[r + 3 * r]) - rotationMatrix[idx + 3 * idx]) + 1.0F);
        rVec(0) = 0.0F;
        rVec(1) = 0.0F;
        rVec(2) = 0.0F;
        rVec(iidx) = b_s / 2.0F;
        rVec(r) = (rotationMatrix[r + 3 * iidx] + rotationMatrix[iidx + 3 * r]) / (2.0F * b_s);
        rVec(idx) = (rotationMatrix[idx + 3 * iidx] + rotationMatrix[iidx + 3 * idx]) / (2.0F * b_s);
        b_s = 1.29246971E-26F;
        absxk = std::fabs(rVec(0));
        if (absxk > 1.29246971E-26F)
        {
            y = 1.0F;
            b_s = absxk;
        }
        else
        {
            t = absxk / 1.29246971E-26F;
            y = t * t;
        }

        absxk = std::fabs(rVec(1));
        if (absxk > b_s)
        {
            t = b_s / absxk;
            y = y * t * t + 1.0F;
            b_s = absxk;
        }
        else
        {
            t = absxk / b_s;
            y += t * t;
        }

        absxk = std::fabs(rVec(2));
        if (absxk > b_s)
        {
            t = b_s / absxk;
            y = y * t * t + 1.0F;
            b_s = absxk;
        }
        else
        {
            t = absxk / b_s;
            y += t * t;
        }

        y = b_s * std::sqrt(y);
        rVec(0) = theta * rVec(0) / y;
        rVec(1) = theta * rVec(1) / y;
        rVec(2) = theta * rVec(2) / y;
    }
    return rVec;
}

Mat34<double> MathFun::InverseKinematics(const Mat34<double> &pfoot, const Mat34<double> &proll)
{
    Vec4<bool> isFR;
    auto &qrParam = GetQrParam();
    isFR << true, false, true, false;
    Mat34<double> p = pfoot - proll;
    Mat34<double> q;
    q.setZero();
    double sigma1, sigma2, sigma3, Hipy, Hipz, angleUp;
    double L0 = qrParam.LEG_L0;
    double L1 = qrParam.LEG_L1;
    double L2 = qrParam.LEG_L2;

    double legLength;
    double legMax = sqrt(pow(qrParam.LEG_L0, 2) + pow(qrParam.LEG_L1 + qrParam.LEG_L2, 2)) - 0.02; // 最大腿长限制
    // check value right or not
    for (int i = 0; i < 4; i++)
    {
        legLength = sqrt(pow(p(0, i), 2) + pow(p(1, i), 2) + pow(p(2, i), 2));
        if (legLength > legMax)
        {
            p(0, i) *= (legMax / legLength); // 按照超出比例缩小位置矢量的值，使得反解有效,否则qCmd = Nan
            p(1, i) *= (legMax / legLength);
            p(2, i) *= (legMax / legLength);
        }
    }

    // 交叉腿，多解切换了造成突变
    if (p(1, 3) > -(qrParam.LEG_L0 + 0.001) && p(1, 3) < -(qrParam.LEG_L0 - 0.001)) // > -0.071  < -0.069
    {
        p(1, 3) = -(qrParam.LEG_L0 - 0.001); // -0.069
                                             // cout << "modify 3 !!" << endl;
    }

    if (p(1, 0) > (qrParam.LEG_L0 - 0.001) && p(1, 0) < (qrParam.LEG_L0 + 0.001)) // > 0.069  < 0.071
    {
        p(1, 0) = (qrParam.LEG_L0 - 0.001); // 0.069
                                            // cout << "modify 0 !!" << endl;
    }

    if (p(1, 1) > -(qrParam.LEG_L0 + 0.001) && p(1, 1) < -(qrParam.LEG_L0 - 0.001)) // > -0.071  < -0.069
    {
        p(1, 1) = -(qrParam.LEG_L0 - 0.001); // -0.069
                                             // cout << "modify 1 !!" << endl;
    }

    if (p(1, 2) > (qrParam.LEG_L0 - 0.001) && p(1, 2) < (qrParam.LEG_L0 + 0.001)) // > 0.069  < 0.071
    {
        p(1, 2) = (qrParam.LEG_L0 - 0.001); // 0.069
                                            // cout << "modify 2 !!" << endl;
    }

    // 斜率不存在时,数值计算不出现？
    for (int i = 0; i < 4; i++)
    {
        if (isFR(i) == true)
        {
            L0 = -qrParam.LEG_L0;
        }
        else
        {
            L0 = qrParam.LEG_L0;
        }

        sigma3 = sqrt(-pow(L0, 2) + pow(p(1, i), 2) + pow(p(2, i), 2));

        // cout << "sigma3 = " << sigma3 << endl;

        sigma2 = pow(p(1, i), 2) * pow(p(2, i), 2) + pow(p(1, i), 4) - pow(L0, 2) * pow(p(1, i), 2) + pow(L0, 2) * pow(p(2, i), 2) - 2 * L0 * p(1, i) * p(2, i) * sigma3;

        // cout << "sigma2 = " << sigma2 << endl;

        sigma1 = pow(L0, 4) * p(2, i) + L0 * pow(p(1, i), 3) * sigma3 - pow(L0, 3) * p(1, i) * sigma3 - pow(L0, 2) * pow(p(1, i), 2) * p(2, i);

        // cout << "sigma1 = " << sigma1 << endl;

        Hipy = -(pow(L0, 2) * p(2, i) + (pow(p(1, i), 2) * sigma1 / sigma2) - L0 * p(1, i) * sigma3 - (pow(L0, 2) * sigma1 / sigma2)) / (L0 * sigma3 - p(1, i) * p(2, i));
        // cout << "Hipy = " << Hipy << endl;

        Hipz = (pow(L0, 4) * p(2, i) + L0 * pow(p(1, i), 3) * sigma3 - pow(L0, 3) * p(1, i) * sigma3 - pow(L0, 2) * pow(p(1, i), 2) * p(2, i)) /
               (pow(p(1, i), 2) * pow(p(2, i), 2) + pow(p(1, i), 4) - pow(L0, 2) * pow(p(1, i), 2) + pow(L0, 2) * pow(p(2, i), 2) - 2 * L0 * p(1, i) * p(2, i) * sigma3);
        // cout << "Hipz = " << Hipz << endl;
        //  atan2 值域为（-pi,pi），转换成(0,2*pi), 再转换成我的坐标系定义
        if (isFR(i) == true)
            q(0, i) = MathFun::atan2toRoll(atan2(Hipz, Hipy)) - M_PI; // https://blog.csdn.net/weixin_42142612/article/details/80972768
        else
            q(0, i) = atan2(Hipz, Hipy);
        // cout << "Hipy = " << Hipy << endl;
        //  double legL = sqrt(pow(p(0, i), 2) + pow((p(1, i) - Hipy), 2) + pow((p(2, i) - Hipz), 2));
        double legL = sqrt(pow(p(0, i), 2) + pow((p(1, i) - Hipy), 2) + pow((p(2, i) - Hipz), 2));

        // cout << "legL = " << legL << endl;

        if (legL <= qrParam.ikLegRange.min && ikErrFlag == false)
        {
            LOG_WARN("legL = {}, out of work space ! from IK!", legL);
            legL = qrParam.ikLegRange.min;
            ikErrFlag = true;
        }
        else if (legL >= qrParam.ikLegRange.max && ikErrFlag == false)
        {
            LOG_WARN("legL = {}, out of work space ! from IK!", legL);
            legL = qrParam.ikLegRange.max;
            ikErrFlag = true;
        }

        q(2, i) = M_PI - acos((pow(L1, 2) + pow(L2, 2) - pow(legL, 2)) / (2 * L1 * L2)); // 求三角形内角，再求补角即为关节角
        angleUp = acos((pow(L1, 2) + pow(legL, 2) - pow(L2, 2)) / (2 * L1 * legL));      // 三角形上顶角
        q(1, i) = asin(p(0, i) / legL) - angleUp + M_PI / 2;                             // 转换成我的坐标系
    }

    return q;
}

Mat34<double> MathFun::JacobianT(const Mat34<double> &f, const Mat34<double> &q)
{
    auto &qrParam = GetQrParam();
    Vec4<bool> isFR;
    isFR << true, false, true, false;
    Vec3<double> Q;
    Mat3<double> JacobianT;
    Mat34<double> tau;
    Q.setZero();
    JacobianT.setZero();
    tau.setZero();
    for (int i = 0; i < 4; i++)
    {
        Q = q.col(i);
        double sigma3 = cos(Q(1) + Q(2));
        double sigma2 = cos(Q(1)) * qrParam.LEG_L1 + sigma3 * qrParam.LEG_L2;
        double sigma1 = sin(Q(1)) * qrParam.LEG_L1 + sin(Q(1) + Q(2)) * qrParam.LEG_L2;
        if (isFR[i] == true)
        {
            JacobianT << 0.0, sin(Q(0)) * qrParam.LEG_L0 + cos(Q(0)) * sigma1, sin(Q(0)) * sigma1 - cos(Q(0)) * qrParam.LEG_L0, sigma1, sin(Q(0)) * sigma2, -cos(Q(0)) * sigma2,
                sin(Q(1) + Q(2)) * qrParam.LEG_L2, sin(Q(0)) * sigma3 * qrParam.LEG_L2, -cos(Q(0)) * sigma3 * qrParam.LEG_L2;
        }
        else
        {
            JacobianT << 0.0, sin(Q(0)) * qrParam.LEG_L0 * -1.0 + cos(Q(0)) * sigma1, sin(Q(0)) * sigma1 - cos(Q(0)) * qrParam.LEG_L0 * -1.0, sigma1, sin(Q(0)) * sigma2,
                -cos(Q(0)) * sigma2, sin(Q(1) + Q(2)) * qrParam.LEG_L2, sin(Q(0)) * sigma3 * qrParam.LEG_L2, -cos(Q(0)) * sigma3 * qrParam.LEG_L2;
        }

        tau.col(i) = JacobianT * f.col(i);
    }
    // cout << "JacobianT =  " << JacobianT << endl;
    return tau;
}

Mat34<double> MathFun::JacobianInv(const Mat34<double> &v, const Mat34<double> &q)
{
    auto &qrParam = GetQrParam();
    Vec4<bool> isFR;
    isFR << true, false, true, false;
    Vec3<double> Q;
    Mat3<double> JacobianT;
    Mat34<double> w;
    Q.setZero();
    JacobianT.setZero();
    w.setZero();
    for (int i = 0; i < 4; i++)
    {
        Q = q.col(i);
        double sigma3 = cos(Q(1) + Q(2));
        double sigma2 = cos(Q(1)) * qrParam.LEG_L1 + sigma3 * qrParam.LEG_L2;
        double sigma1 = sin(Q(1)) * qrParam.LEG_L1 + sin(Q(1) + Q(2)) * qrParam.LEG_L2;
        if (isFR[i] == true)
        {
            JacobianT << 0.0, sin(Q(0)) * qrParam.LEG_L0 + cos(Q(0)) * sigma1, sin(Q(0)) * sigma1 - cos(Q(0)) * qrParam.LEG_L0, sigma1, sin(Q(0)) * sigma2, -cos(Q(0)) * sigma2,
                sin(Q(1) + Q(2)) * qrParam.LEG_L2, sin(Q(0)) * sigma3 * qrParam.LEG_L2, -cos(Q(0)) * sigma3 * qrParam.LEG_L2;
        }
        else
        {
            JacobianT << 0.0, sin(Q(0)) * qrParam.LEG_L0 * -1.0 + cos(Q(0)) * sigma1, sin(Q(0)) * sigma1 - cos(Q(0)) * qrParam.LEG_L0 * -1.0, sigma1, sin(Q(0)) * sigma2,
                -cos(Q(0)) * sigma2, sin(Q(1) + Q(2)) * qrParam.LEG_L2, sin(Q(0)) * sigma3 * qrParam.LEG_L2, -cos(Q(0)) * sigma3 * qrParam.LEG_L2;
        }

        w.col(i) = JacobianT.transpose().inverse() * v.col(i);
    }
    return w;
}

Vec3<double> MathFun::SwingDynamics(Vec3<double> q, Vec3<double> w, Vec3<double> a, int legIndex)
{
    auto &qrParam = GetQrParam();
    // 经过检查，左右腿只是L0相差负号而已20221209
    // 0 2  右腿  1  3 左腿
    double s1, s2, s3, c1, c2, c3, m1, m2, m3, l0, l1, l2, Izz1, Izz2, Izz3, pc0, pc1, pc2, g;
    g = G(2);

    // Izz1 = 1219741.36 * 10e-9;
    // Izz2 = 14927212.1 * 10e-9;
    // Izz3 = 532279.76 * 10e-9;

    Izz1 = 0;
    Izz2 = 0;
    Izz3 = 0;

    pc0 = 1.0;
    pc1 = 0.154;
    pc2 = 0.234;

    m1 = qrParam.DOG_M_LEG00;
    m2 = qrParam.DOG_M_LEG01;
    m3 = qrParam.DOG_M_LEG02;

    if (legIndex == 0 || legIndex == 2)
    {
        l0 = qrParam.LEG_L0;
    }
    else
    {
        l0 = -qrParam.LEG_L0;
    }

    l1 = qrParam.LEG_L1;
    l2 = qrParam.LEG_L2;

    s1 = sin(q(0));
    s2 = sin(q(1));
    s3 = sin(q(2));
    c1 = cos(q(0));
    c2 = cos(q(1));
    c3 = cos(q(2));

    Vec3<double> tau, V, G;
    Mat3<double> M;

    // 假设质心在连杆末端
    /*
    M(0, 0) = -2 * m2 * (pow(l2, 2) * pow(c3, 2) + l1 * l2 * c3 + pow(l1, 2) - 0.5 * pow(l2, 2)) * pow(c2, 2) + 2 * s2 * s3 * l2 * m2 * (l2 * c3 + l1) * c2 + pow(c3, 2) * pow(l2,
    2) * m2 + 2 * c3 * l1 * l2 * m2 + (2 * (pow(l0, 2) + pow(l1, 2))) * m2 + pow(l0, 2) * m1 + Izz1; M(0, 1) = ((l2 * c3 + 2 * l1) * c2 - l2 * s2 * s3) * m2 * l0;  //
    相差负号，左右腿 M(0, 2) = l0 * m2 * (c2 * c3 - s2 * s3) * l2;                  // 相差负号

    M(1, 0) = ((l2 * c3 + 2 * l1) * c2 - l2 * s2 * s3) * m2 * l0;  // 相差负号
    M(1, 1) = 2 * m2 * (l1 * l2 * c3 + pow(l1, 2) + 0.5 * (pow(l2, 2))) + Izz2;
    M(1, 2) = m2 * (l1 * c3 + l2) * l2;

    M(2, 0) = l0 * m2 * (c2 * c3 - s2 * s3) * l2;  // 相差负号
    M(2, 1) = m2 * (l1 * c3 + l2) * l2;
    M(2, 2) = pow(l2, 2) * m2 + Izz3;

    V(0) = 4 * ((s3 * l2 * (l2 * c3 + l1) * pow(c2, 2) + s2 * (pow(l2, 2) * pow(c3, 2) + l1 * l2 * c3 + pow(l1, 2) - 0.5 * (pow(l2, 2))) * c2 - 0.5 * (s3 * l2 * (l2 * c3 + l1))) *
    w(1) + l2 * (s3 * (l2 * c3 + 0.5 * l1) * pow(c2, 2) + c2 * s2 * (pow(c3, 2) * l2 + 0.5 * (l1 * c3 - l2)) - 0.5 * (s3 * (l2 * c3 + l1))) * w(2)) * m2 * w(0) - (s3 * l2 * c2 +
    (l2 * c3 + 2 * l1) * s2) * l0 * m2 * pow(w(1), 2) - 2 * w(2) * l0 * l2 * m2 * (s2 * c3 + c2 * s3) * w(1) - l0 * l2 * m2 * (s2 * c3 + c2 * s3) * pow(w(2), 2);  // 相差负号 V(1)
    = -2 * m2 * (s3 * l2 * (l2 * c3 + l1) * pow(c2, 2) + s2 * (pow(l2, 2) * pow(c3, 2) + l1 * l2 * c3 + pow(l1, 2) - 0.5 * (pow(l2, 2))) * c2 - 0.5 * (s3 * l2 * (l2 * c3 + l1))) *
    pow(w(0), 2) - l1 * l2 * s3 * pow(w(2), 2) * m2 - 2 * l1 * l2 * s3 * w(1) * w(2) * m2; V(2) = -l2 * (2 * m2 * (s3 * (l2 * c3 + 0.5 * l1) * pow(c2, 2) + c2 * s2 * (pow(c3, 2) *
    l2 + 0.5 * (l1 * c3 - l2)) - 0.5 * s3 * (l2 * c3 + l1)) * pow(w(0), 2) - l1 * s3 * pow(w(1), 2) * m2);

    G(0) = c2 * s1 * s3 * l2 * m3 + c3 * s1 * s2 * l2 * m3 + s1 * l1 * (m2 + m3) * s2 - l0 * c1 * (m1 + m2 + m3);  // L0 相差负号
    G(1) = -c1 * ((c3 * l2 * m3 + l1 * (m2 + m3)) * c2 - s2 * s3 * l2 * m3);
    G(2) = -c1 * m3 * (c2 * c3 - s2 * s3) * l2;
    */

    // 考虑质心不在连杆末端
    M(0, 0) = -2 * m2 * (pow(pc2, 2) * pow(l2, 2) * pow(c3, 2) + pc2 * l1 * l2 * c3 + 0.5 * (pow(l1, 2) * (pow(pc1, 2) + 1)) - 0.5 * (pow(l2, 2) + pow(pc2, 2))) * pow(c2, 2) +
              2 * s2 * s3 * l2 * m2 * pc2 * (pc2 * l2 * c3 + l1) * c2 + pow(c3, 2) * pow(l2, 2) * m2 * pow(pc2, 2) + 2 * c3 * pc2 * l1 * l2 * m2 +
              (2 * (pow(l0, 2) + pow(l1, 2) * (pow(pc1, 2) + 1))) * m2 + pow(l0, 2) * m1 * pow(pc0, 2) + Izz1;
    M(0, 1) = ((pc2 * l2 * c3 + l1 * (pc1 + 1)) * c2 - pc2 * l2 * s2 * s3) * m2 * l0; // 相差负号，左右腿
    M(0, 2) = l0 * m2 * (c2 * c3 - s2 * s3) * l2 * pc2;                               // 相差负号

    M(1, 0) = ((pc2 * l2 * c3 + l1 * (pc1 + 1)) * c2 - pc2 * l2 * s2 * s3) * m2 * l0; // 相差负号
    M(1, 1) = 2 * l2 * m2 * c3 * pc2 * l1 + (pow(pc2, 2) * pow(l2, 2) + pow(l1, 2) * (pow(pc1, 2) + 1)) * m2 + Izz2;
    M(1, 2) = m2 * (l1 * c3 + pc2 * l2) * l2 * pc2;

    M(2, 0) = l0 * m2 * (c2 * c3 - s2 * s3) * l2 * pc2; // 相差负号
    M(2, 1) = pc2 * m2 * (l1 * c3 + pc2 * l2) * l2;
    M(2, 2) = pow(l2, 2) * pow(pc2, 2) * m2 + Izz3;

    V(0) = 4 *
               ((s3 * pc2 * l2 * (pc2 * l2 * c3 + l1) * pow(c2, 2) +
                 s2 * (pow(pc2, 2) * pow(l2, 2) * pow(c3, 2) + pc2 * l1 * l2 * c3 - 0.5 * (pow(pc2, 2) * pow(l2, 2)) + 0.5 * (pow(l1, 2) * (pow(pc1, 2) + 1))) * c2 -
                 0.5 * (s3 * pc2 * l2 * (pc2 * l2 * c3 + l1))) *
                    w(1) +
                pc2 * l2 *
                    (s3 * (pc2 * l2 * c3 + 0.5 * l1) * pow(c2, 2) + c2 * s2 * (pow(c3, 2) * pow(pc2, 2) * l2 + 0.5 * (l1 * c3 - pc2 * l2)) - 0.5 * (s3 * (pc2 * l2 * c3 + l1))) *
                    w(2)) *
               m2 * w(0) -
           (s3 * l2 * pc2 * c2 + (pc2 * l2 * c3 + l1 * (pc1 + 1)) * s2) * l0 * m2 * pow(w(1), 2) - 2 * w(2) * pc2 * l0 * l2 * m2 * (s2 * c3 + c2 * s3) * w(1) -
           pc2 * l0 * l2 * m2 * (s2 * c3 + c2 * s3) * pow(w(2), 2); // 相差负号
    V(1) = -2 * m2 *
               (s3 * pc2 * l2 * (pc2 * l2 * c3 + l1) * pow(c2, 2) +
                s2 * (pow(pc2, 2) * pow(l2, 2) * pow(c3, 2) + pc2 * l1 * l2 * c3 - 0.5 * (pow(pc2, 2) * pow(l2, 2)) + 0.5 * (pow(l1, 2) * (pow(pc1, 2) + 1))) * c2 -
                0.5 * (s3 * pc2 * l2 * (pc2 * l2 * c3 + l1))) *
               pow(w(0), 2) -
           l1 * l2 * s3 * pow(w(2), 2) * pc2 * m2 - 2 * pc2 * l1 * l2 * s3 * w(1) * w(2) * m2;
    V(2) =
        -pc2 * l2 *
        (2 * m2 * (s3 * (pc2 * l2 * c3 + 0.5 * l1) * pow(c2, 2) + c2 * s2 * (pow(c3, 2) * pc2 * l2 + 0.5 * (l1 * c3 - pc2 * l2)) - 0.5 * s3 * (pc2 * l2 * c3 + l1)) * pow(w(0), 2) -
         l1 * s3 * pow(w(1), 2) * m2);

    G(0) = c2 * s1 * s3 * l2 * pc2 * m3 + s1 * (m3 * c3 * pc2 * l2 + l1 * (pc1 * m2 + m3)) * s2 - l0 * c1 * (m1 * pc0 + m2 + m3); // L0 相差负号
    G(1) = -((m3 * c3 * pc2 * l2 + l1 * (pc1 * m2 + m3)) * c2 - m3 * s2 * s3 * pc2 * l2) * c1;
    G(2) = -c1 * m3 * (c2 * c3 - s2 * s3) * l2 * pc2;

    tau = M * a + V + G * g;
    return tau;
}
Mat34<double> MathFun::forwardKinematics(Mat34<double> q)
{
    auto &qrParam = GetQrParam();
    Vec4<bool> isFR;
    isFR << true, false, true, false;
    Mat34<double> p;
    for (int i = 0; i < 4; i++)
    {
        p(0, i) = -qrParam.LEG_L1 * cos(q(1, i)) - qrParam.LEG_L2 * cos(q(1, i) + q(2, i));
        if (!isFR(i))
        {
            p(1, i) = (-qrParam.LEG_L1 * sin(q(1, i)) - qrParam.LEG_L2 * sin(q(1, i) + q(2, i))) * -sin(q(0, i)) + qrParam.LEG_L0 * cos(q(0, i));
            p(2, i) = (-qrParam.LEG_L1 * sin(q(1, i)) - qrParam.LEG_L2 * sin(q(1, i) + q(2, i))) * cos(q(0, i)) + qrParam.LEG_L0 * sin(q(0, i));
        }
        else
        {
            p(1, i) = (-qrParam.LEG_L1 * sin(q(1, i)) - qrParam.LEG_L2 * sin(q(1, i) + q(2, i))) * -sin(q(0, i)) - qrParam.LEG_L0 * cos(q(0, i));
            p(2, i) = (-qrParam.LEG_L1 * sin(q(1, i)) - qrParam.LEG_L2 * sin(q(1, i) + q(2, i))) * cos(q(0, i)) - qrParam.LEG_L0 * sin(q(0, i));
        }
    }
    return p;
}

Mat34<double> MathFun::InverseKinematics_new(const Mat34<double> &pfoot, const Mat34<double> &proll)
{
    Vec4<bool> isFR;
    auto &qrParam = GetQrParam();
    isFR << true, false, true, false;
    // Mat34<double> p = pfoot - proll;
    Mat34<double> p = pfoot - proll;
    Mat34<double> q;
    q.setZero();
    // double sigma1, sigma2, sigma3, Hipy, Hipz, angleUp;
    double theta1_offset, lxzp, alfa_xzp, theta_yz, theta_yz_p, theta_yz_pp, dyz, lyz, n, alfa_off, alfa1;
    double L0 = qrParam.LEG_L0;
    double L1 = qrParam.LEG_L1;
    double L2 = qrParam.LEG_L2;
    // 最大腿长限制
    double legLength;
    double legMax = sqrt(pow(qrParam.LEG_L0, 2) + pow(qrParam.LEG_L1 + qrParam.LEG_L2, 2)) - 0.0004;
    // check value right or not
    for (int i = 0; i < 4; i++)
    {
        legLength = sqrt(pow(p(0, i), 2) + pow(p(1, i), 2) + pow(p(2, i), 2));
        if (legLength > legMax)
        {
            p(0, i) *= (legMax / legLength); // 按照超出比例缩小位置矢量的值，使得反解有效,否则qCmd = Nan
            p(1, i) *= (legMax / legLength);
            p(2, i) *= (legMax / legLength);
        }
    }

    // 检查是否落入虚拟墙
    double deltaX[2] = {0.0895, 0.1695};
    for (int i = 0; i < 4; i++)
    {
        if (isFR(i) == true)
        {
            double deltaY[2] = {-0.11, 0.02};
            double deltaZ[2] = {-0.09, 0.1};
            if (p(0, i) >= deltaX[0] && p(0, i) <= deltaX[1] && p(1, i) >= deltaY[0] && p(1, i) <= deltaY[1] && p(2, i) >= deltaZ[0] && p(2, i) <= deltaZ[1])
            {
                if (p(0, i) > deltaX[0])
                {
                    p(0, i) = deltaX[0];
                }
                if (p(0, i) < deltaX[1])
                {
                    p(0, i) = deltaX[1];
                }
                if (p(1, i) > deltaY[0])
                {
                    p(1, i) = deltaY[0];
                }
                if (p(1, i) < deltaY[1])
                {
                    p(1, i) = deltaY[1];
                }
                if (p(2, i) > deltaZ[0])
                {
                    p(2, i) = deltaZ[0];
                }
                if (p(2, i) < deltaZ[1])
                {
                    p(2, i) = deltaZ[1];
                }
            }
        }
        else
        {
            double deltaY[2] = {-0.02, 0.11};
            double deltaZ[2] = {-0.1, 0.09};
            if (p(0, i) >= deltaX[0] && p(0, i) <= deltaX[1] && p(1, i) >= deltaY[0] && p(1, i) <= deltaY[1] && p(2, i) >= deltaZ[0] && p(2, i) <= deltaZ[1])
            {
                if (p(0, i) > deltaX[0])
                {
                    p(0, i) = deltaX[0];
                }
                if (p(0, i) < deltaX[1])
                {
                    p(0, i) = deltaX[1];
                }
                if (p(1, i) > deltaY[0])
                {
                    p(1, i) = deltaY[0];
                }
                if (p(1, i) < deltaY[1])
                {
                    p(1, i) = deltaY[1];
                }
                if (p(2, i) > deltaZ[0])
                {
                    p(2, i) = deltaZ[0];
                }
                if (p(2, i) < deltaZ[1])
                {
                    p(2, i) = deltaZ[1];
                }
            }
        }
    }
    // 求解
    for (int i = 0; i < 4; i++)
    {
        dyz = sqrt(pow(p(1, i), 2) + (pow(p(2, i), 2)));
        lyz = sqrt(pow(dyz, 2) - (pow(L0, 2)));
        theta1_offset = atan(L0 / lyz);
        /*求解∠1*/
        if (isFR(i) == true)
        {                                    /*right*/
            if (p(1, i) >= 0 && p(2, i) < 0) /*第四象限*/
            {
                theta_yz_pp = atan(p(1, i) / -p(2, i));
                q(0, i) = theta_yz_pp + theta1_offset;
            }
            else if (p(1, i) < 0 && p(2, i) >= 0) /*第二象限*/
            {
                theta_yz_p = atan(p(2, i) / -p(1, i));
                q(0, i) = -(theta_yz_p + (M_PI_2 - theta1_offset));
            }
            else if (p(1, i) <= 0 && p(2, i) < 0) /*第三象限*/
            {
                theta_yz = atan(p(1, i) / p(2, i));
                q(0, i) = -(theta_yz - theta1_offset);
            }
            else
            {
                theta_yz = atan(p(1, i) / p(2, i));
                q(0, i) = -(theta_yz + theta1_offset);
            }
        }
        else
        {
            if (p(1, i) <= 0 && p(2, i) < 0)
            {
                theta_yz = atan(-p(1, i) / -p(2, i));
                q(0, i) = -(theta_yz + theta1_offset);
            }
            else if (p(1, i) > 0 && p(2, i) >= 0)
            {
                theta_yz_p = atan(p(2, i) / p(1, i));
                q(0, i) = theta_yz_p + (M_PI_2 - theta1_offset);
            }
            else if (p(1, i) >= 0 && p(2, i) < 0)
            {
                theta_yz = atan(p(1, i) / -p(2, i));
                q(0, i) = -(theta1_offset - theta_yz);
            }
            else
            {
                theta_yz = atan(-p(1, i) / p(2, i));
                q(0, i) = theta1_offset + theta_yz;
            }
        }
        /*求解∠3*/
        lxzp = sqrt(pow(lyz, 2) + pow(p(0, i), 2));
        n = ((pow(lxzp, 2) - pow(L2, 2) - pow(L1, 2))) / (2 * L1);
        q(2, i) = acos(n / L2);
        /*求解∠2*/
        alfa_xzp = -atan(p(0, i) / lyz);
        alfa_off = acos((L1 + n) / lxzp);
        alfa1 = alfa_xzp + alfa_off;
        q(1, i) = M_PI_2 - alfa1;
    }
    return q;
}

bool MathFun::IsOdd(int u)
{
    if (u % 2 == 0)
    {
        return true;
    }
    else
    {
        return false;
    }
}

Mat3<double> MathFun::quat2RotMat(Vec4<double> Vec)
{
    double e0 = Vec(0);
    double e1 = Vec(1);
    double e2 = Vec(2);
    double e3 = Vec(3);

    Mat3<double> R;
    R << 1 - 2 * (e2 * e2 + e3 * e3), 2 * (e1 * e2 - e0 * e3), 2 * (e1 * e3 + e0 * e2), 2 * (e1 * e2 + e0 * e3), 1 - 2 * (e1 * e1 + e3 * e3), 2 * (e2 * e3 - e0 * e1),
        2 * (e1 * e3 - e0 * e2), 2 * (e2 * e3 + e0 * e1), 1 - 2 * (e1 * e1 + e2 * e2);
    return R;
}

Vec3<double> MathFun::quat2RPY(Vec4<double> q)
{
    Vec3<double> rpy;
    double as = min(-2. * (q[1] * q[3] - q[0] * q[2]), .99999);
    rpy(2) = atan2(2 * (q[1] * q[2] + q[0] * q[3]), pow(q[0], 2) + pow(q[1], 2) - pow(q[2], 2) - pow(q[3], 2));
    rpy(1) = asin(as);
    rpy(0) = atan2(2 * (q[2] * q[3] + q[0] * q[1]), pow(q[0], 2) - pow(q[1], 2) - pow(q[2], 2) + pow(q[3], 2));
    return rpy;
}

double MathFun::NormalizeAngle(const double angle)
{
    double a = std::fmod(angle + M_PI, 2.0 * M_PI);
    if (a < 0.0)
    {
        a += (2.0 * M_PI);
    }
    return a - M_PI;
}

Vec3<double> MathFun::QuatRotateInverse(Vec4<double> quat, Vec3<double> vec)
{
    double q_w = quat(0);
    Vec3<double> q_vec;
    q_vec << quat(1), quat(2), quat(3);

    Vec3<double> a = vec * (2.0 * q_w * q_w - 1.0);
    Vec3<double> b = q_vec.cross(vec) * q_w * 2.0;
    Vec3<double> c = q_vec * (q_vec.transpose() * vec) * 2;
    return a - b + c;
}
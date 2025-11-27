
#pragma once
#include <set>

#include "baseline.hpp"
class MathFun
{
public:
    MathFun();

    static double Sign(double value);
    static double Poly5(double T, double t);
    static double changeAtan2Range(double atan2Value);
    static double deg2rad(double deg);
    static double rad2deg(double rad);
    Vec3<double> CycloidCurve(const Vec3<double>& p0, const Vec3<double>& p1, double h, double t, double T);
    Vec3<double> BezierCurve7(const Vec3<double>& p0, const Vec3<double>& p1, const Vec3<double>& pA, const Vec3<double>& pB, const Vec3<double>& pC, const Vec3<double>& pD,
                              const Vec3<double>& pE, double s, double maxS);
    static Vec3<double> BezierCurve5(const Vec3<double>& p0, const Vec3<double>& p1, const Vec3<double>& pA, const Vec3<double>& pB, const Vec3<double>& pC, double s, double maxS);
    static Vec3<double> BezierCurve4(const Vec3<double>& p0, const Vec3<double>& p1, const Vec3<double>& pA, const Vec3<double>& pB, double s, double maxS);
    static Vec3<double> BezierCurve3(const Vec3<double>& p0, const Vec3<double>& p1, const Vec3<double>& pA, double s, double maxS);
    static Vec3<double> BezierCurve2(const Vec3<double>& p0, const Vec3<double>& p1, double s, double maxS);
    //  static Vec3<double> rotm2eul(Mat3<double> R);
    //  static Mat3<double> eul2rotm(double roll, double pitch, double yaw);
    static Vec3<double> Rx(const Vec3<double>& pIn, double radX);
    static Vec3<double> Ry(const Vec3<double>& pIn, double radY);
    static Vec3<double> Rz(const Vec3<double>& pIn, double radZ);
    static double atan2toRoll(double value);
    // 添加static为静态函数，操作过程必须是临时变量,不操作任何自己的或者其他的变量，使用前加 Math::
    static Vec3<double> quaternion2Euler(const Vec4<double>& q);

    static double setValueRange(double value, double rangeA, double rangeB, bool isPrint, int ErrIndex);
    double SetValueRange(double value, double min, double max, const std::string& str, bool isPrint = true);
    Mat3<double> rotVec2rotMat(Vec3<double> rVec);

    Vec3<double> rotMat2rotVec(Mat3<double> R);

    Mat3<double> quat2RotMat(Vec4<double> Vec);
    Vec3<double> quat2RPY(Vec4<double> q);
    void xswap(double x[9], int ix0, int iy0);
    void svd(const double A[9], double U[9], double s[3], double V[9]);

    bool IsOdd(int u);

    // 以下几个函数为Matlab生成的计算边界值所需，写在此处，无需重复写
    void rt_InitInfAndNaN();      // 初始化边界值
    bool rtIsInf(double value);   // 计算
    bool rtIsInfF(double value);  // 计算
    bool rtIsNaN(double value);   // 计算
    bool rtIsNaNF(double value);  // 计算

    double rtInf;
    double rtMinusInf;
    double rtNaN;
    double rtInfF;
    double rtMinusInfF;
    double rtNaNF;

    Mat34<double> InverseKinematics(const Mat34<double>& pfoot, const Mat34<double>& proll);
    Mat34<double> JacobianT(const Mat34<double>& matrixDescartes, const Mat34<double>& q);
    Mat34<double> JacobianInv(const Mat34<double>& matrixDescartes, const Mat34<double>& q);
    Mat34<double> forwardKinematics(Mat34<double> q);
    Mat34<double> InverseKinematics_new(const Mat34<double>& pfoot, const Mat34<double>& proll);

    Vec3<double> SwingDynamics(Vec3<double> q, Vec3<double> w, Vec3<double> a, int legIndex);
    double NormalizeAngle(const double angle);
    Vec3<double> QuatRotateInverse(Vec4<double> quat, Vec3<double> vec);

private:
    bool ikErrFlag;
    std::set<std::string> setValueRangePrint_;
};
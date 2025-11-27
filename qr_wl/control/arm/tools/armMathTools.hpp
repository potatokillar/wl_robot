
#pragma once
#include <set>

#include "ArmType.hpp"
#include "armParam.hpp"
#include "baseline.hpp"
class ArmMathFun
{
public:
    ArmMathFun();

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

    void quat_squad(double q[8], double s, double val[4]);
    void do_slerp(const double q1[4], double q2[4], double t, double q_out[4]);

    Pose Fkine(Vec6<double> Radian, Pose ToolValue);
    std::pair<Pose, double> Fkine_Phi(std::vector<double> Radian);
    std::pair<bool, Vec6<double>> Ikine(Pose KPS6, Vec6<double> qReward, Pose ToolValue);
    Quaternion Rpy2quat(Rpy rpy);
    Rpy Quat2rpy(Quaternion quat);

    PlanResults Plan5(double S, double planT, double t);
    Mat4<double> Cartesian2RotMat(Pose pos);
    Pose RotMat2Cartesian(Mat4<double> KPS44);

    double CumputeQs(Quaternion quat_start, Quaternion quat_end);
    Quaternion QuatInterpolate(double Y, double S, Quaternion qs, Quaternion qe);

    Mat4<double> Ti(double theta, double d, double a, double alpha);
    Mat4<double> Ti_modified(double theta, double d, double a, double alpha);
    Mat4<double> Ti_standard(double theta, double d, double a, double alpha);

    Vec6<double> Ji(Mat4<double> T);
    Mat6<double> Jacobian_tool6(std::vector<double> q);
    Mat6<double> Jacobian_base6(std::vector<double> q);

    bool CheckAngle(std::vector<double> q, std::vector<double> qmin, std::vector<double> qmax);
    bool CheckAngle_seven(std::vector<double> q, std::vector<double> qmin, std::vector<double> qmax);

    RetState Checkrapid(double set);
    Result<Vec6<double>> CheckSpeed(double UserV, Vec6<double> ParamVmax);
    Result<Vec6<double>> CheckAcc(double UserA, Vec6<double> ParamAmax, MoveFlag flag);

    Result<Vec7<double>> CheckSpeed_seven(double UserV, Vec7<double> ParamVmax);
    Result<Vec7<double>> CheckAcc_seven(double UserA, Vec7<double> ParamAmax, MoveFlag flag);
    std::pair<Pose, double> Fkine_seven(Vec7<double> Radian, Pose ToolValue);
    std::pair<bool, Vec7<double>> IKine_seven(Pose KPS6, double phi, Vec7<double> qRev);
    int sign(double x);
    Mat3<double> Vec2DiagMat(Vec6<double> input);
    std::pair<Mat3<double>, Vec3<double>> RotMat2RP(Mat4<double> input);
    Vec6<double> Dynamic(Vec6<double> q, Vec6<double> qdot, Vec6<double> qddot);

private:
    bool ikErrFlag;
    std::set<std::string> setValueRangePrint_;
};
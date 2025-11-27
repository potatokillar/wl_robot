
#include <gtest/gtest.h>

#include "baseline.hpp"
#include "mathTools.hpp"
#include "quadrupedParam.hpp"

using namespace ::std;

bool Mat34DoubleIsEqual(const Mat34<double>& lhs, const Mat34<double>& rhs, double near = 0.0001)
{
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 4; j++) {
            //   EXPECT_NEAR(lhs(i, j), rhs(i, j), near);
            if (fabs(lhs(i, j) - rhs(i, j)) > near) {
                printf("i:%d, j:%d, lhs:%f, rhs:%f\n", i, j, lhs(i, j), rhs(i, j));  // todo日志架构修改后用日志
                return false;
            }
        }
    }
    return true;
}

// 构造函数和析构函数，本类构造和析构时执行一次
// SetUp/TearDown函数，每个TEST_F都会执行一次
class MathTools_config_base : public ::testing::Test
{
protected:
    MathTools_config_base()
    {
        GetLog().SetLevel(6);
        GetRobotCfgFile().ParseFile("./config/00-base.toml");
    }
    ~MathTools_config_base() override {}
    void SetUp() override {}
    void TearDown() override {}
};

TEST_F(MathTools_config_base, forwardKinematics)
{
    MathFun mathFun;

    Mat34<double> P_Check;
    Mat34<double> q;

    /*正常值*/
    q << 0.2, 0.2, 0.2, 0.2, 0.3, 0.3, 0.3, 0.3, 0.4, 0.4, 0.4, 0.4;
    Mat34<double> P_Foot = mathFun.forwardKinematics(q);
    // cout << "P_Foot:" << P_Foot << endl;
    P_Check << -0.344, -0.344, -0.344, -0.344, -0.03127, 0.1059, -0.03127, 0.1059, -0.1981, -0.1703, -0.1981, -0.1703;
    EXPECT_TRUE(Mat34DoubleIsEqual(P_Foot, P_Check));

    q << 0.2, 0.2, 0.2, 0.2, 0.6, 0.6, 0.6, 0.6, 0.1, 0.1, 0.1, 0.1;
    P_Foot = mathFun.forwardKinematics(q);
    P_Check << -0.318, -0.318, -0.318, -0.318, -0.02057, 0.1166, -0.02057, 0.1166, -0.2509, -0.223, -0.2509, -0.223;
    EXPECT_TRUE(Mat34DoubleIsEqual(P_Foot, P_Check));

    q << 0.1, 0.1, 0.1, 0.1, 0.5, 0.5, 0.5, 0.5, 0.7, 0.7, 0.7, 0.7;
    P_Foot = mathFun.forwardKinematics(q);
    P_Check << -0.248, -0.248, -0.248, -0.248, -0.04147, 0.09783, -0.04147, 0.09783, -0.2879, -0.2739, -0.2879, -0.2739;
    EXPECT_TRUE(Mat34DoubleIsEqual(P_Foot, P_Check));

    q << 0, 0, 0, 0, 0.7, 0.7, 0.7, 0.7, 0.6, 0.6, 0.6, 0.6;
    P_Foot = mathFun.forwardKinematics(q);
    P_Check << -0.2065, -0.2065, -0.2065, -0.2065, -0.07, 0.07, -0.07, 0.07, -0.3216, -0.3216, -0.3216, -0.3216;
    EXPECT_TRUE(Mat34DoubleIsEqual(P_Foot, P_Check));

    q << 0.3, 0.3, 0.3, 0.3, 0.5, 0.5, 0.5, 0.5, 0.7, 0.7, 0.7, 0.7;
    P_Foot = mathFun.forwardKinematics(q);
    P_Check << -0.248, -0.248, -0.248, -0.248, 0.01655, 0.1503, 0.01655, 0.1503, -0.2904, -0.249, -0.2904, -0.249;
    EXPECT_TRUE(Mat34DoubleIsEqual(P_Foot, P_Check));

    q << 0.05, 0.05, 0.05, 0.05, 0.6, 0.6, 0.6, 0.6, 0.2, 0.2, 0.2, 0.2;
    P_Foot = mathFun.forwardKinematics(q);
    P_Check << -0.3044, -0.3044, -0.3044, -0.3044, -0.0571, 0.08273, -0.0571, 0.08273, -0.2596, -0.2526, -0.2596, -0.2526;
    EXPECT_TRUE(Mat34DoubleIsEqual(P_Foot, P_Check));

    q << 0.04, 0.04, 0.04, 0.04, 0.5, 0.5, 0.5, 0.5, 0.2, 0.2, 0.2, 0.2;
    P_Foot = mathFun.forwardKinematics(q);
    P_Check << -0.3285, -0.3285, -0.3285, -0.3285, -0.06096, 0.07893, -0.06096, 0.07893, -0.2273, -0.2217, -0.2273, -0.2217;
    EXPECT_TRUE(Mat34DoubleIsEqual(P_Foot, P_Check));

    q << 0.06, 0.06, 0.06, 0.06, 0.5, 0.5, 0.5, 0.5, 0.07, 0.07, 0.07, 0.07;
    P_Foot = mathFun.forwardKinematics(q);
    P_Check << -0.3439, -0.3439, -0.3439, -0.3439, -0.05765, 0.0821, -0.05765, 0.0821, -0.2076, -0.1992, -0.2076, -0.1992;
    EXPECT_TRUE(Mat34DoubleIsEqual(P_Foot, P_Check));

    q << 0, 0, 0, 0, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4;
    P_Foot = mathFun.forwardKinematics(q);
    P_Check << -0.3236, -0.3236, -0.3236, -0.3236, -0.07, 0.07, -0.07, 0.07, -0.2214, -0.2214, -0.2214, -0.2214;
    EXPECT_TRUE(Mat34DoubleIsEqual(P_Foot, P_Check));

    q << 0.05, 0.05, 0.05, 0.05, 0.4, 0.4, 0.4, 0.4, 0.9, 0.9, 0.9, 0.9;
    ;
    P_Foot = mathFun.forwardKinematics(q);
    P_Check << -0.2377, -0.2377, -0.2377, -0.2377, -0.05639, 0.08344, -0.05639, 0.08344, -0.2738, -0.2668, -0.2738, -0.2668;
    EXPECT_TRUE(Mat34DoubleIsEqual(P_Foot, P_Check));

    /*边界值*/
    q << 1.57, 1.57, 1.57, 1.57, 0, 0, 0, 0, 0, 0, 0, 0;
    P_Foot = mathFun.forwardKinematics(q);
    P_Check << -0.4, -0.4, -0.4, -0.4, -5.574e-05, 5.574e-05, -5.574e-05, 5.574e-05, -0.07, 0.07, -0.07, 0.07;
    EXPECT_TRUE(Mat34DoubleIsEqual(P_Foot, P_Check));

    q << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    P_Foot = mathFun.forwardKinematics(q);
    P_Check << -0.4, -0.4, -0.4, -0.4, -0.07, 0.07, -0.07, 0.07, -0, 0, -0, 0;
    EXPECT_TRUE(Mat34DoubleIsEqual(P_Foot, P_Check));

    q << 0, 0, 0, 0, 1.57, 1.57, 1.57, 1.57, 1.57, 1.57, 1.57, 1.57;
    P_Foot = mathFun.forwardKinematics(q);
    P_Check << 0.1998, 0.1998, 0.1998, 0.1998, -0.07, 0.07, -0.07, 0.07, -0.2003, -0.2003, -0.2003, -0.2003;
    EXPECT_TRUE(Mat34DoubleIsEqual(P_Foot, P_Check));

    q << 0, 0, 0, 0, 0, 0, 0, 0, 1.57, 1.57, 1.57, 1.57;
    P_Foot = mathFun.forwardKinematics(q);
    P_Check << -0.2002, -0.2002, -0.2002, -0.2002, -0.07, 0.07, -0.07, 0.07, -0.2, -0.2, -0.2, -0.2;
    EXPECT_TRUE(Mat34DoubleIsEqual(P_Foot, P_Check));

    q << -1.57, -1.57, -1.57, -1.57, 0, 0, 0, 0, 0, 0, 0, 0;
    P_Foot = mathFun.forwardKinematics(q);
    P_Check << -0.4, -0.4, -0.4, -0.4, -5.574e-05, 5.574e-05, -5.574e-05, 5.574e-05, 0.07, -0.07, 0.07, -0.07;
    EXPECT_TRUE(Mat34DoubleIsEqual(P_Foot, P_Check));

    q << -1.57, -1.57, -1.57, -1.57, -1.57, -1.57, -1.57, -1.57, 0, 0, 0, 0;
    P_Foot = mathFun.forwardKinematics(q);
    P_Check << -0.0003185, -0.0003185, -0.0003185, -0.0003185, 0.3999, 0.4001, 0.3999, 0.4001, 0.07032, -0.06968, 0.07032, -0.06968;
    EXPECT_TRUE(Mat34DoubleIsEqual(P_Foot, P_Check));

    q << 3.14, 3.14, 3.14, 3.14, 6.28, 6.28, 6.28, 6.28, 0.0001, 0.0001, 0.0001, 0.0001;
    P_Foot = mathFun.forwardKinematics(q);
    P_Check << -0.4, -0.4, -0.4, -0.4, 0.07, -0.07, 0.07, -0.07, -0.001366, -0.001143, -0.001366, -0.001143;
    EXPECT_TRUE(Mat34DoubleIsEqual(P_Foot, P_Check));

    q << -3.14, -3.14, -3.14, -3.14, 0.57, 0.57, 0.57, 0.57, 1.57, 1.57, 1.57, 1.57;
    P_Foot = mathFun.forwardKinematics(q);
    P_Check << -0.06059, -0.06059, -0.06059, -0.06059, 0.06956, -0.07044, 0.06956, -0.07044, 0.2765, 0.2763, 0.2765, 0.2763;
    EXPECT_TRUE(Mat34DoubleIsEqual(P_Foot, P_Check));

    q << 0, 0, 0, 0, 3.14, 3.14, 3.14, 3.14, 3.14, 3.14, 3.14, 3.14;
    P_Foot = mathFun.forwardKinematics(q);
    P_Check << 7.61e-07, 7.61e-07, 7.61e-07, 7.61e-07, -0.07, 0.07, -0.07, 0.07, 0.0003185, 0.0003185, 0.0003185, 0.0003185;
    EXPECT_TRUE(Mat34DoubleIsEqual(P_Foot, P_Check));

    q << 3.14, 3.14, 3.14, 3.14, 0, 0, 0, 0, 0, 0, 0, 0;
    P_Foot = mathFun.forwardKinematics(q);
    P_Check << -0.4, -0.4, -0.4, -0.4, 0.07, -0.07, 0.07, -0.07, -0.0001115, 0.0001115, -0.0001115, 0.0001115;
    EXPECT_TRUE(Mat34DoubleIsEqual(P_Foot, P_Check));
}
// 逆解测试中正常值也未通过，查代码确定一下问题，或替换成新代码

TEST_F(MathTools_config_base, InverseKinematics_new)
{
    MathFun mathFun;
    Mat34<double> q1;
    q1 << -1.0471, -1.0471, -1.0471, -1.0471, 0, 0, 0, 0, 0.6282, 0.6282, 0.6282, 0.6282;
    Mat34<double> q;
    Mat34<double> q_Check;
    Mat34<double> P_foot = mathFun.forwardKinematics(q1);
    // P_foot << -0.3618, -0.3618, -0.3618, -0.3618, -0.1368, -0.0668, -0.1368, -0.0668, 0.0018, -0.1194, 0.0018, -0.1194;
    Mat34<double> P_roll;
    P_roll.setZero();
    // cout << "P_foot=" << P_foot << endl;
    auto q2 = mathFun.InverseKinematics_new(P_foot, P_roll);
    q_Check << -1.0471, -1.0471, -1.0471, -1.0471, 0, 0, 0, 0, 0.6282, 0.6282, 0.6282, 0.6282;
    EXPECT_TRUE(Mat34DoubleIsEqual(q2, q_Check));

    // cout << "q1=" << q << endl;

    q1 << -1.0472, -1.0472, -1.0472, -1.0472, 0.2230, 0.2230, 0.2230, 0.2230, 0.8339, 0.8339, 0.8339, 0.8339;
    P_foot = mathFun.forwardKinematics(q1);
    q2 = mathFun.InverseKinematics_new(P_foot, P_roll);
    q_Check << -1.0472, -1.0472, -1.0472, -1.0472, 0.2230, 0.2230, 0.2230, 0.2230, 0.8339, 0.8339, 0.8339, 0.8339;
    EXPECT_TRUE(Mat34DoubleIsEqual(q2, q_Check));

    /*常规值*/

    q1 << 0.05, 0.05, 0.05, 0.05, 0.4, 0.4, 0.4, 0.4, 0.9, 0.9, 0.9, 0.9;
    P_foot = mathFun.forwardKinematics(q1);
    q2 = mathFun.InverseKinematics_new(P_foot, P_roll);
    q_Check << 0.05, 0.05, 0.05, 0.05, 0.4, 0.4, 0.4, 0.4, 0.9, 0.9, 0.9, 0.9;
    EXPECT_TRUE(Mat34DoubleIsEqual(q2, q_Check));

    q1 << 0, 0, 0, 0, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4;
    P_foot = mathFun.forwardKinematics(q1);
    q2 = mathFun.InverseKinematics_new(P_foot, P_roll);
    q_Check << 0, 0, 0, 0, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4;
    EXPECT_TRUE(Mat34DoubleIsEqual(q2, q_Check));

    q1 << 0.2, 0.2, 0.2, 0.2, 0.3, 0.3, 0.3, 0.3, 0.4, 0.4, 0.4, 0.4;
    P_foot = mathFun.forwardKinematics(q1);
    q2 = mathFun.InverseKinematics_new(P_foot, P_roll);
    q_Check << 0.2, 0.2, 0.2, 0.2, 0.3, 0.3, 0.3, 0.3, 0.4, 0.4, 0.4, 0.4;
    EXPECT_TRUE(Mat34DoubleIsEqual(q2, q_Check));

    q1 << 0, 0, 0, 0, 1.57, 1.57, 1.57, 1.57, 1.57, 1.57, 1.57, 1.57;
    P_foot = mathFun.forwardKinematics(q1);
    q2 = mathFun.InverseKinematics_new(P_foot, P_roll);
    q_Check << 0, 0, 0, 0, 1.57, 1.57, 1.57, 1.57, 1.57, 1.57, 1.57, 1.57;
    EXPECT_TRUE(Mat34DoubleIsEqual(q2, q_Check));

    q1 << 0, 0, 0, 0, 1.57, 1.57, 1.57, 1.57, 0.2, 0.2, 0.2, 0.2;
    P_foot = mathFun.forwardKinematics(q1);
    q2 = mathFun.InverseKinematics_new(P_foot, P_roll);
    q_Check << 0, 0, 0, 0, 1.57, 1.57, 1.57, 1.57, 0.2, 0.2, 0.2, 0.2;
    EXPECT_TRUE(Mat34DoubleIsEqual(q2, q_Check));

    q1 << 0.05, 0.05, 0.05, 0.05, 0.6, 0.6, 0.6, 0.6, 0.2, 0.2, 0.2, 0.2;
    P_foot = mathFun.forwardKinematics(q1);
    q2 = mathFun.InverseKinematics_new(P_foot, P_roll);
    q_Check << 0.05, 0.05, 0.05, 0.05, 0.6, 0.6, 0.6, 0.6, 0.2, 0.2, 0.2, 0.2;
    EXPECT_TRUE(Mat34DoubleIsEqual(q2, q_Check));

    q1 << 0.1, 0.1, 0.1, 0.1, 0.5, 0.5, 0.5, 0.5, 0.7, 0.7, 0.7, 0.7;
    P_foot = mathFun.forwardKinematics(q1);
    q2 = mathFun.InverseKinematics_new(P_foot, P_roll);
    q_Check << 0.1, 0.1, 0.1, 0.1, 0.5, 0.5, 0.5, 0.5, 0.7, 0.7, 0.7, 0.7;
    EXPECT_TRUE(Mat34DoubleIsEqual(q2, q_Check));

    q1 << 0.2, 0.2, 0.2, 0.2, 0.6, 0.6, 0.6, 0.6, 0.1, 0.1, 0.1, 0.1;
    P_foot = mathFun.forwardKinematics(q1);
    q2 = mathFun.InverseKinematics_new(P_foot, P_roll);
    q_Check << 0.2, 0.2, 0.2, 0.2, 0.6, 0.6, 0.6, 0.6, 0.1, 0.1, 0.1, 0.1;
    EXPECT_TRUE(Mat34DoubleIsEqual(q2, q_Check));

    q1 << 0, 0, 0, 0, 0.7, 0.7, 0.7, 0.7, 0.6, 0.6, 0.6, 0.6;
    P_foot = mathFun.forwardKinematics(q1);
    q2 = mathFun.InverseKinematics_new(P_foot, P_roll);
    q_Check << 0, 0, 0, 0, 0.7, 0.7, 0.7, 0.7, 0.6, 0.6, 0.6, 0.6;
    EXPECT_TRUE(Mat34DoubleIsEqual(q2, q_Check));

    /*有机会用到的特殊值*/

    q1 << -1.0472, -1.0472, -1.0472, -1.0472, 0.2230, 0.2230, 0.2230, 0.2230, 2.2728, 2.2728, 2.2728, 2.2728;
    P_foot = mathFun.forwardKinematics(q1);
    q2 = mathFun.InverseKinematics_new(P_foot, P_roll);
    q_Check << -1.0472, -1.0472, -1.0472, -1.0472, 0.2230, 0.2230, 0.2230, 0.2230, 2.2728, 2.2728, 2.2728, 2.2728;
    EXPECT_TRUE(Mat34DoubleIsEqual(q2, q_Check));

    q1 << -1.0472, -1.0472, -1.0472, -1.0472, 1.1151, 1.1151, 1.1151, 1.1151, 1.0394, 1.0394, 1.0394, 1.0394;
    P_foot = mathFun.forwardKinematics(q1);
    q2 = mathFun.InverseKinematics_new(P_foot, P_roll);
    q_Check << -1.0472, -1.0472, -1.0472, -1.0472, 1.1151, 1.1151, 1.1151, 1.1151, 1.0394, 1.0394, 1.0394, 1.0394;
    EXPECT_TRUE(Mat34DoubleIsEqual(q2, q_Check));

    q1 << -1.0472, -1.0472, -1.0472, -1.0472, 1.1151, 1.1151, 1.1151, 1.1151, 2.4784, 2.4784, 2.4784, 2.4784;
    P_foot = mathFun.forwardKinematics(q1);
    q2 = mathFun.InverseKinematics_new(P_foot, P_roll);
    q_Check << -1.0472, -1.0472, -1.0472, -1.0472, 1.1151, 1.1151, 1.1151, 1.1151, 2.4784, 2.4784, 2.4784, 2.4784;
    EXPECT_TRUE(Mat34DoubleIsEqual(q2, q_Check));

    q1 << -1.0472, -1.0472, -1.0472, -1.0472, 0.1357, 0.1357, 0.1357, 0.1357, 2.4784, 2.4784, 2.4784, 2.4784;
    P_foot = mathFun.forwardKinematics(q1);
    q2 = mathFun.InverseKinematics_new(P_foot, P_roll);
    q_Check << -1.0472, -1.0472, -1.0472, -1.0472, 0.1357, 0.1357, 0.1357, 0.1357, 2.4784, 2.4784, 2.4784, 2.4784;
    EXPECT_TRUE(Mat34DoubleIsEqual(q2, q_Check));

    q1 << -1.0472, -1.0472, -1.0472, -1.0472, 0.1357, 0.1357, 0.1357, 0.1357, 2.7925, 2.7925, 2.7925, 2.7925;
    P_foot = mathFun.forwardKinematics(q1);
    q2 = mathFun.InverseKinematics_new(P_foot, P_roll);
    q_Check << -1.0472, -1.0472, -1.0472, -1.0472, 0.1357, 0.1357, 0.1357, 0.1357, 2.7925, 2.7925, 2.7925, 2.7925;
    EXPECT_TRUE(Mat34DoubleIsEqual(q2, q_Check));

    q1 << -0.8436, -0.8436, -0.8436, -0.8436, 1.9199, 1.9199, 1.9199, 1.9199, 1.8617, 1.8617, 1.8617, 1.8617;
    P_foot = mathFun.forwardKinematics(q1);
    q2 = mathFun.InverseKinematics_new(P_foot, P_roll);
    q_Check << -0.8436, -0.8436, -0.8436, -0.8436, 1.9199, 1.9199, 1.9199, 1.9199, 1.8617, 1.8617, 1.8617, 1.8617;
    EXPECT_TRUE(Mat34DoubleIsEqual(q2, q_Check));

    q1 << -0.8436, -0.8436, -0.8436, -0.8436, 1.9199, 1.9199, 1.9199, 1.9199, 2.1720, 2.1720, 2.1720, 2.1720;
    P_foot = mathFun.forwardKinematics(q1);
    q2 = mathFun.InverseKinematics_new(P_foot, P_roll);
    q_Check << -0.8436, -0.8436, -0.8436, -0.8436, 1.9199, 1.9199, 1.9199, 1.9199, 2.1720, 2.1720, 2.1720, 2.1720;
    EXPECT_TRUE(Mat34DoubleIsEqual(q2, q_Check));

    q1 << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    P_foot = mathFun.forwardKinematics(q1);
    q2 = mathFun.InverseKinematics_new(P_foot, P_roll);
    q_Check << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    EXPECT_TRUE(Mat34DoubleIsEqual(q2, q_Check));

    /*异常值*/
    q1 << -1.0472, -1.0472, -1.0472, -1.0472, 2.0071, 2.0071, 2.0071, 2.0071, 2.2728, 2.2728, 2.2728, 2.2728;
    P_foot = mathFun.forwardKinematics(q1);
    q2 = mathFun.InverseKinematics_new(P_foot, P_roll);
    q_Check << -0.9539, 1.7415, -0.9539, 1.7415, 1.6392, 1.4926, 1.6392, 1.4926, 2.1799, 2.0371, 2.1799, 2.0371;
    EXPECT_TRUE(Mat34DoubleIsEqual(q2, q_Check));

    q1 << -0.3491, -0.3491, -0.3491, -0.3491, 1.7841, 1.7841, 1.7841, 1.7841, 2.2728, 2.2728, 2.2728, 2.2728;
    P_foot = mathFun.forwardKinematics(q1);
    q2 = mathFun.InverseKinematics_new(P_foot, P_roll);
    q_Check << -0.9539, 1.7415, -0.9539, 1.7415, 1.6392, 1.4926, 1.6392, 1.4926, 2.1799, 2.0371, 2.1799, 2.0371;
    EXPECT_TRUE(Mat34DoubleIsEqual(q2, q_Check));

    q1 << -0.3491, -0.3491, -0.3491, -0.3491, 1.7841, 1.7841, 1.7841, 1.7841, 2.4784, 2.4784, 2.4784, 2.4784;
    P_foot = mathFun.forwardKinematics(q1);
    q2 = mathFun.InverseKinematics_new(P_foot, P_roll);
    q_Check << -0.9539, 1.7415, -0.9539, 1.7415, 1.6392, 1.4926, 1.6392, 1.4926, 2.1799, 2.0371, 2.1799, 2.0371;
    EXPECT_TRUE(Mat34DoubleIsEqual(q2, q_Check));

    q1 << -0.1164, -0.1164, -0.1164, -0.1164, 2.0071, 2.0071, 2.0071, 2.0071, 2.2728, 2.2728, 2.2728, 2.2728;
    P_foot = mathFun.forwardKinematics(q1);
    q2 = mathFun.InverseKinematics_new(P_foot, P_roll);
    q_Check << -0.9539, 1.7415, -0.9539, 1.7415, 1.6392, 1.4926, 1.6392, 1.4926, 2.1799, 2.0371, 2.1799, 2.0371;
    EXPECT_TRUE(Mat34DoubleIsEqual(q2, q_Check));

    q1 << -0.1164, -0.1164, -0.1164, -0.1164, 2.0071, 2.0071, 2.0071, 2.0071, 2.4784, 2.4784, 2.4784, 2.4784;
    P_foot = mathFun.forwardKinematics(q1);
    q2 = mathFun.InverseKinematics_new(P_foot, P_roll);
    q_Check << -0.9539, 1.7415, -0.9539, 1.7415, 1.6392, 1.4926, 1.6392, 1.4926, 2.1799, 2.0371, 2.1799, 2.0371;
    EXPECT_TRUE(Mat34DoubleIsEqual(q2, q_Check));

    q1 << -0.1164, -0.1164, -0.1164, -0.1164, 2.0071, 2.0071, 2.0071, 2.0071, 2.2728, 2.2728, 2.2728, 2.2728;
    P_foot = mathFun.forwardKinematics(q1);
    q2 = mathFun.InverseKinematics_new(P_foot, P_roll);
    q_Check << -0.9539, 1.7415, -0.9539, 1.7415, 1.6392, 1.4926, 1.6392, 1.4926, 2.1799, 2.0371, 2.1799, 2.0371;
    EXPECT_TRUE(Mat34DoubleIsEqual(q2, q_Check));

    q1 << -0.1164, -0.1164, -0.1164, -0.1164, 1.5611, 1.5611, 1.5611, 1.5611, 2.2728, 2.2728, 2.2728, 2.2728;
    P_foot = mathFun.forwardKinematics(q1);
    q2 = mathFun.InverseKinematics_new(P_foot, P_roll);
    q_Check << -0.9539, 1.7415, -0.9539, 1.7415, 1.6392, 1.4926, 1.6392, 1.4926, 2.1799, 2.0371, 2.1799, 2.0371;
    EXPECT_TRUE(Mat34DoubleIsEqual(q2, q_Check));

    q1 << -0.1164, -0.1164, -0.1164, -0.1164, 1.5611, 1.5611, 1.5611, 1.5611, 2.4784, 2.4784, 2.4784, 2.4784;
    P_foot = mathFun.forwardKinematics(q1);
    q2 = mathFun.InverseKinematics_new(P_foot, P_roll);
    q_Check << -0.9539, 1.7415, -0.9539, 1.7415, 1.6392, 1.4926, 1.6392, 1.4926, 2.1799, 2.0371, 2.1799, 2.0371;
    EXPECT_TRUE(Mat34DoubleIsEqual(q2, q_Check));

    q1 << -0.1164, -0.1164, -0.1164, -0.1164, 2.0071, 2.0071, 2.0071, 2.0071, 2.2728, 2.2728, 2.2728, 2.2728;
    P_foot = mathFun.forwardKinematics(q1);
    q2 = mathFun.InverseKinematics_new(P_foot, P_roll);
    q_Check << -0.9539, 1.7415, -0.9539, 1.7415, 1.6392, 1.4926, 1.6392, 1.4926, 2.1799, 2.0371, 2.1799, 2.0371;
    EXPECT_TRUE(Mat34DoubleIsEqual(q2, q_Check));

    q1 << -0.1164, -0.1164, -0.1164, -0.1164, 1.7841, 1.7841, 1.7841, 1.7841, 2.4784, 2.4784, 2.4784, 2.4784;
    P_foot = mathFun.forwardKinematics(q1);
    q2 = mathFun.InverseKinematics_new(P_foot, P_roll);
    q_Check << -0.9539, 1.7415, -0.9539, 1.7415, 1.6392, 1.4926, 1.6392, 1.4926, 2.1799, 2.0371, 2.1799, 2.0371;
    EXPECT_TRUE(Mat34DoubleIsEqual(q2, q_Check));

    q1 << -0.1164, -0.1164, -0.1164, -0.1164, 1.7841, 1.7841, 1.7841, 1.7841, 2.2728, 2.2728, 2.2728, 2.2728;
    P_foot = mathFun.forwardKinematics(q1);
    q2 = mathFun.InverseKinematics_new(P_foot, P_roll);
    q_Check << -0.9539, 1.7415, -0.9539, 1.7415, 1.6392, 1.4926, 1.6392, 1.4926, 2.1799, 2.0371, 2.1799, 2.0371;
    EXPECT_TRUE(Mat34DoubleIsEqual(q2, q_Check));
}

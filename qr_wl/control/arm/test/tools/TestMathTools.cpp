
#include <gtest/gtest.h>

#include "armCore.hpp"
#include "armMathTools.hpp"
#include "armParam.hpp"
#include "baseline.hpp"

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
class MathTools_config_arm : public ::testing::Test
{
protected:
    void SetUp() override
    {
        // RobotLogSetOutput(LogPosi::none);
        GetRobotCfgFile().ParseFile("./config/arm-iris.toml");
        ArmParam::GetInstance().ReInit();
    }
    void TearDown() override {}
};

TEST_F(MathTools_config_arm, forwardKinematics)
{
    cout << "test arm" << endl;

    ArmMathFun math;

    // Vec6<double> q, IK, qRecv;
    std::string Tool = "tool1";
    cout << "FK" << endl;

    // Mat4<double> FK, posend;
    // q << 0, 0, 0.2, 0, 0, 0;
    // // q << 0, 0.6, 1.2, 0, 1.1, 0;
    // qRecv = 0.99 * q;
    ArmUserParam user_;
    user_.userTool = GetArmParam().coor.tool;

    vector<string> key;
    for (const auto& pair : user_.userTool) {
        key.push_back(pair.first);
        // const Pose& value = pair.second;
        // 处理 key 和 value
    }
    for (size_t i = 0; i < key.size(); i++) {
        cout << "Tool:" << key[i] << endl;
    }
    // cout << "Tool:" << key[0] << endl;
    Pose FK;
    Vec6<double> q;
    // Pose tool = {0, 0, 0.243, 0, 0, 0};
    Pose Tool0 = {0, 0, 0, 0, 0, 0};
    // q << -0.26180641174316405, -0.3842949676513672, -1.5187335205078125, -0.00011978149414062501, -1.2042352294921876, -1.396328811645508;
    q << 0.0, 0.8, 0.8, 0, 1.57, 0;
    Pose PoseFK = math.Fkine(q, Tool0);

    PoseFK.PrintPose();

    Pose pos1 = {0.438254, 1.00139e-05, 0.146714, -3.13156, 0.037809, 9.06719e-06};

    auto IK = math.Ikine(pos1, 0.99 * q, Tool0);

    cout << "IK:" << IK.first << endl;
    // Pose POSEND{0.2, -0.1, 0.2, 1.57, 1.57, 3.14};  // XYZ rpy 24.3.14 matlab工具箱验证通过
    // Pose POSEND{-0.2, 0.1, 0.2, -1.57, 1.57, 1.57};
    // Mat4<double> FK;
    // FK = math.Fkine(q, Tool);
    // // FK = math.Cartesian2RotMat(POSEND);
    // cout << "FK:" << FK << endl;
    // posend << 0.999597, -1.73851e-18, -0.0284035, 0.443192, 1.73921e-18, -1, 1.22415e-16, 1.91291e-17, -0.0284035, -1.22415e-16, -0.999597, 0.279597, 0, 0, 0, 1;
    // IK = math.Ikine(FK, qRecv, Tool);
    // cout << "IK:" << IK << endl;

    // Vec6<double> q11, check;
    // q11 << 0.1111, 1245, 0, 3.141592653, -3.141592653, 0;
    // cout << "check:" << check << endl;
}

TEST_F(MathTools_config_arm, check)
{
    ArmMathFun math;

    // double UserV = 6.0;
    // Result<Vec6<double>> UserSpeed;
    // UserSpeed = math.CheckSpeed(UserV, GetArmParam().axis.V_max);
    // if (UserSpeed.first == RetState::ok) {
    //     cout << UserSpeed.second << endl;
    // }
}

// TEST_F(MathTools_config_arm, jacobian)
// {
//     cout << "test cppsql" << endl;
//     ArmMathFun math;

//     vector<Vec6<double>> jacobian;
//     vector<double> q;
//     for (int i = 0; i < 6; i++) {
//         q.push_back(0);
//     }
//     jacobian = math.Jacobian_tool(q);
//     for (int i = 0; i < jacobian.size(); i++) {
//         cout << "jacobian_base:" << jacobian[i] << endl;
//     }
//     q.clear();
//     for (int i = 0; i < 6; i++) {
//         q.push_back(1);
//     }
//     jacobian = math.Jacobian_tool(q);
//     for (int i = 0; i < jacobian.size(); i++) {
//         cout << "jacobian_base:" << jacobian[i] << endl;
//     }
// }
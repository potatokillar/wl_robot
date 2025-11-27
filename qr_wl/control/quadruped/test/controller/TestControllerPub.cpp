

#include <gtest/gtest.h>

#include "baseline.hpp"
#include "controller.hpp"
#include "quadrupedParam.hpp"

// 构造函数和析构函数，本类构造和析构时执行一次
// SetUp/TearDown函数，每个TEST_F都会执行一次
class Controller_config_base : public ::testing::Test
{
protected:
    void SetUp() override
    {
        GetLog().SetLevel(6);
        GetRobotCfgFile().ParseFile("./config/11-gazebo.toml");
        QuadrupedParam::GetInstance().ReInit();
    }
    void TearDown() override {}
};

TEST_F(Controller_config_base, SetLinearVelocity)
{
    BootArgs args;
    args.isSim = true;
    args.noMotor = false;

    Controller contr;
    contr.Start(args);
    auto range = contr.GetLinearVelocityRange();
    // 输入极小值极大值，均通过
    auto ret = contr.SetLinearVelocity(range.x.min, range.y.min, range.z.min);
    EXPECT_EQ(ret, RetState::ok);
    ret = contr.SetLinearVelocity(range.x.max, range.y.max, range.z.max);
    EXPECT_EQ(ret, RetState::ok);
    ret = contr.SetLinearVelocityRatio(0.5, -0.5, 1.5);  // 超限比例不会失败
    EXPECT_EQ(ret, RetState::ok);

    // 输入超限值，失败
    ret = contr.SetLinearVelocity(range.x.max + 1, range.y.max + 1, range.z.max + 1);
    EXPECT_NE(ret, RetState::ok);
    EXPECT_EQ(ret, RetState::outRange);

    contr.Stop();
}

TEST_F(Controller_config_base, SetAngularVelocity)
{
    BootArgs args;
    args.isSim = true;
    args.noMotor = false;

    Controller contr;
    contr.Start(args);

    auto range = contr.GetAngularVelocityRange();
    // 输入极小值极大值，均通过
    auto ret = contr.SetAngularVelocity(range.x.min, range.y.min, range.z.min);
    EXPECT_EQ(ret, RetState::ok);
    ret = contr.SetAngularVelocity(range.x.max, range.y.max, range.z.max);
    EXPECT_EQ(ret, RetState::ok);
    ret = contr.SetAngularVelocityRatio(0.5, -0.5, 1.5);
    EXPECT_EQ(ret, RetState::ok);

    // 输入超限值，失败
    ret = contr.SetAngularVelocity(range.x.max + 1, range.y.max + 1, range.z.max + 1);
    EXPECT_NE(ret, RetState::ok);
    EXPECT_EQ(ret, RetState::outRange);

    contr.Stop();
}

TEST_F(Controller_config_base, SetPose)
{
    BootArgs args;
    args.isSim = true;
    args.noMotor = false;

    Controller contr;
    contr.Start(args);

    auto range = contr.GetPoseRange();
    // 输入极小值极大值，均通过
    auto ret = contr.SetPose(range.roll.min, range.pitch.min, range.yaw.min);
    EXPECT_EQ(ret, RetState::ok);
    ret = contr.SetPose(range.roll.max, range.pitch.max, range.yaw.max);
    EXPECT_EQ(ret, RetState::ok);
    ret = contr.SetPoseRatio(0.5, -0.5, 1.5);
    EXPECT_EQ(ret, RetState::ok);

    // 输入超限值，失败
    ret = contr.SetPose(range.roll.max + 1, range.pitch.max + 1, range.yaw.max + 1);
    EXPECT_NE(ret, RetState::ok);
    EXPECT_EQ(ret, RetState::outRange);

    contr.Stop();
}

TEST_F(Controller_config_base, SetHeight)
{
    BootArgs args;
    args.isSim = true;
    args.noMotor = false;

    Controller contr;
    contr.Start(args);

    auto range = contr.GetHeightRange();
    // 输入极小值极大值，均通过
    auto ret = contr.SetHeight(range.min);
    EXPECT_EQ(ret, RetState::ok);
    ret = contr.SetHeight(range.max);
    EXPECT_EQ(ret, RetState::ok);
    ret = contr.SetHeightRatio(1.5);
    EXPECT_EQ(ret, RetState::ok);

    // 输入超限值，失败
    ret = contr.SetHeight(range.max + 1);
    EXPECT_NE(ret, RetState::ok);
    EXPECT_EQ(ret, RetState::outRange);

    contr.Stop();
}

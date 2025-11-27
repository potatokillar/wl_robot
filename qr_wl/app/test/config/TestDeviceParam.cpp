
#include <gtest/gtest.h>

#include "baseline.hpp"
#include "deviceParam.hpp"

// 构造函数和析构函数，本类构造和析构时执行一次
// SetUp/TearDown函数，每个TEST_F都会执行一次
class TestDeviceParam : public ::testing::Test
{
protected:
    TestDeviceParam()
    {
        // RobotLogSetOutput(LogPosi::none);
    }
    ~TestDeviceParam() override {}
    void SetUp() override {}
    void TearDown() override {}
};

// EXPECT失败，本TEST用例继续执行下一个语句
// ASSERT失败，跳出本TEST用例
// 以上均不影响其它用例
// 仿真狗的参数
TEST_F(TestDeviceParam, qr)
{
    GetRobotCfgFile().ParseFile("./config/qr-gazebo.toml");
    DevQrParam::GetInstance().ReInit();  // 测试用例必须
    auto para = GetDevQrParam();
    EXPECT_EQ(para.model, DevModel::qr::base);
}

// 连杆V2_3
TEST_F(TestDeviceParam, linkV2_3)
{
    GetRobotCfgFile().ParseFile("./config/qr-linkV2-3.toml");
    DevQrParam::GetInstance().ReInit();
    auto para = GetDevQrParam();
    EXPECT_EQ(para.model, DevModel::qr::linkV2_3);
    EXPECT_EQ(para.audioName, "hw:Headphones");
    EXPECT_EQ(para.spiSpeed, 3000000);
}

// 中型
TEST_F(TestDeviceParam, middle)
{
    GetRobotCfgFile().ParseFile("./config/qr-middle.toml");
    DevQrParam::GetInstance().ReInit();
    auto para = GetDevQrParam();
    EXPECT_EQ(para.model, DevModel::qr::middle);
    EXPECT_EQ(para.qrMotorCfg.motorModel(0, 0), MotorModel::tMotor_80_64);
    EXPECT_EQ(para.audioName, "hw:Headphones");
    EXPECT_EQ(para.spiSpeed, 3000000);
}


#include <gtest/gtest.h>

#include "baseline.hpp"
#include "quadrupedParam.hpp"

// 构造函数和析构函数，本类构造和析构时执行一次
// SetUp/TearDown函数，每个TEST_F都会执行一次
class TestQuadrupedParam : public ::testing::Test
{
protected:
    TestQuadrupedParam() { GetLog().SetLevel(6); }
    ~TestQuadrupedParam() override {}
    void SetUp() override {}
    void TearDown() override {}
};

// EXPECT失败，本TEST用例继续执行下一个语句
// ASSERT失败，跳出本TEST用例
// 以上均不影响其它用例
// 仿真狗的参数
TEST_F(TestQuadrupedParam, base)
{
    GetRobotCfgFile().ParseFile("./config/00-base.toml");
    QuadrupedParam::GetInstance().ReInit();  // 测试用例必须
    auto para = GetQrParam();
    EXPECT_EQ(para.model.back(), QrModel::unused);
}

// 连杆V2-3
TEST_F(TestQuadrupedParam, linkV2_3)
{
    GetRobotCfgFile().ParseFile("./config/14-linkV2-3.toml");
    QuadrupedParam::GetInstance().ReInit();
    auto para = GetQrParam();
    EXPECT_EQ(para.model.back(), QrModel::linkV2_3);
    EXPECT_DOUBLE_EQ(para.pComBias(0), 0.0);
    EXPECT_DOUBLE_EQ(para.pComBias(1), 0.0);
    EXPECT_DOUBLE_EQ(para.pComBias(2), 0.0);
}

// 中型
TEST_F(TestQuadrupedParam, middle)
{
    GetRobotCfgFile().ParseFile("./config/06-middle.toml");
    QuadrupedParam::GetInstance().ReInit();
    auto para = GetQrParam();
    EXPECT_EQ(para.model.back(), QrModel::middle);
}


#include <gtest/gtest.h>

#include <memory>

#include "baseline.hpp"
#include "device/config_base.hpp"
#include "qrMotor.hpp"
#include "robotConfig.hpp"
#include "spi2can.hpp"
#include "stub.h"

using namespace std;

static SPI_TO_CAN_S sendBuf[2];
static SPI_TO_CAN_S recvBuf[2];
/**
 * @description: 桩函数
 * @param *pThis 替换类成员函数时，第一个参数是this指针
 * @param board
 * @param &msg
 * @return {}
 */
void stub_AddMessage(void *pThis, int board, const SPI_TO_CAN_S &msg)
{
    UNUSED(pThis);
    sendBuf[board] = msg;
}

SPI_TO_CAN_S stub_GetMessage(void *pThis, int board)
{
    UNUSED(pThis);
    return recvBuf[board];
}
void stub_Transfer() {}
/**
 * @description: SPI_TO_CAN_S的比较函数
 * @param &lhs
 * @param &rhs
 * @return {}
 */
bool Spi2CanIsSame(const SPI_TO_CAN_S &lhs, const SPI_TO_CAN_S &rhs)
{
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 8; j++) {
            if (lhs.payload.data[i][j] != rhs.payload.data[i][j]) {
                printf("%d %d %X\n", i, j, lhs.payload.data[i][j]);
                return false;
            }
        }
    }
    return true;
}

class TestMotorProtocol : public ::testing::Test
{
protected:
    TestMotorProtocol()
    {
        RobotLogSetOutput(LogPosi::none);
        GetRobotCfgFile().ParseFile("./config/00-base.toml");
        DeviceParam::GetInstance().ReInit();
        // 对底层函数打桩
        stub.set(ADDR(SpiToCan, AddMessage), stub_AddMessage);
        stub.set(ADDR(SpiToCan, GetMessage), stub_GetMessage);
        stub.set(ADDR(SpiToCan, Transfer), stub_Transfer);
    }
    ~TestMotorProtocol() override {}
    void SetUp() override {}
    void TearDown() override {}

private:
    Stub stub;
};

TEST_F(TestMotorProtocol, MotorProtocolMit_Send)
{
    SubCfg_QrMotor motorCfg;
    motorCfg.motorModel.fill(MotorModel::haitai);
    motorCfg.chiral.fill(false);
    auto motorProtocol = make_unique<MotorProtocolMit>(motorCfg);

    msg::qr::motor_cmd cmd;
    SPI_TO_CAN_S ret1, ret2;

    // 0号腿给全0值，默认是0不需要写
    ret1.payload.leg_right_abad[0] = 0x7F;
    ret1.payload.leg_right_abad[1] = 0xFF;
    ret1.payload.leg_right_abad[2] = 0x7F;
    ret1.payload.leg_right_abad[3] = 0xF0;
    ret1.payload.leg_right_abad[4] = 0x00;
    ret1.payload.leg_right_abad[5] = 0x00;
    ret1.payload.leg_right_abad[6] = 0x07;
    ret1.payload.leg_right_abad[7] = 0xFF;

    ret1.payload.leg_right_hip[0] = 0x7F;
    ret1.payload.leg_right_hip[1] = 0xFF;
    ret1.payload.leg_right_hip[2] = 0x7F;
    ret1.payload.leg_right_hip[3] = 0xF0;
    ret1.payload.leg_right_hip[4] = 0x00;
    ret1.payload.leg_right_hip[5] = 0x00;
    ret1.payload.leg_right_hip[6] = 0x07;
    ret1.payload.leg_right_hip[7] = 0xFF;

    ret1.payload.leg_right_knee[0] = 0x7F;
    ret1.payload.leg_right_knee[1] = 0xFF;
    ret1.payload.leg_right_knee[2] = 0x7F;
    ret1.payload.leg_right_knee[3] = 0xF0;
    ret1.payload.leg_right_knee[4] = 0x00;
    ret1.payload.leg_right_knee[5] = 0x00;
    ret1.payload.leg_right_knee[6] = 0x07;
    ret1.payload.leg_right_knee[7] = 0xFF;

    // 1号腿给非0的正常值
    cmd.q_abad[1] = 0.478543;
    cmd.q_hip[1] = 0.234497;
    cmd.q_knee[1] = -1.031773;

    cmd.qd_abad[1] = -0.000000;
    cmd.qd_hip[1] = 0.000000;
    cmd.qd_knee[1] = 0.000000;

    cmd.t_abad[1] = 2.152570;
    cmd.t_hip[1] = 0.000000;
    cmd.t_knee[1] = -3.185864;

    cmd.kp_abad[1] = 50;
    cmd.kp_hip[1] = 50;
    cmd.kp_knee[1] = 50;

    cmd.kd_abad[1] = 3;
    cmd.kd_hip[1] = 3;
    cmd.kd_knee[1] = 3;

    ret1.payload.leg_left_abad[0] = 0x80;
    ret1.payload.leg_left_abad[1] = 0xA3;
    ret1.payload.leg_left_abad[2] = 0x7F;
    ret1.payload.leg_left_abad[3] = 0xF1;
    ret1.payload.leg_left_abad[4] = 0x99;
    ret1.payload.leg_left_abad[5] = 0x99;
    ret1.payload.leg_left_abad[6] = 0x98;
    ret1.payload.leg_left_abad[7] = 0xF4;

    ret1.payload.leg_left_hip[0] = 0x80;
    ret1.payload.leg_left_hip[1] = 0x4F;
    ret1.payload.leg_left_hip[2] = 0x7F;
    ret1.payload.leg_left_hip[3] = 0xF1;
    ret1.payload.leg_left_hip[4] = 0x99;
    ret1.payload.leg_left_hip[5] = 0x99;
    ret1.payload.leg_left_hip[6] = 0x97;
    ret1.payload.leg_left_hip[7] = 0xFF;

    ret1.payload.leg_left_knee[0] = 0x7E;
    ret1.payload.leg_left_knee[1] = 0x9D;
    ret1.payload.leg_left_knee[2] = 0x7F;
    ret1.payload.leg_left_knee[3] = 0xF1;
    ret1.payload.leg_left_knee[4] = 0x99;
    ret1.payload.leg_left_knee[5] = 0x99;
    ret1.payload.leg_left_knee[6] = 0x96;
    ret1.payload.leg_left_knee[7] = 0x95;

    // 2号腿给边界值
    cmd.q_abad[2] = -95.5;
    cmd.q_hip[2] = 95.5;
    cmd.q_knee[2] = 0;

    cmd.qd_abad[2] = -45;
    cmd.qd_hip[2] = 45;
    cmd.qd_knee[2] = 0;

    cmd.t_abad[2] = -18;
    cmd.t_hip[2] = 18;
    cmd.t_knee[2] = 0;

    cmd.kp_abad[2] = 0;
    cmd.kp_hip[2] = 500;
    cmd.kp_knee[2] = 0;

    cmd.kd_abad[2] = 0;
    cmd.kd_hip[2] = 5;
    cmd.kd_knee[2] = 0;

    ret2.payload.leg_right_abad[0] = 0x00;
    ret2.payload.leg_right_abad[1] = 0x00;
    ret2.payload.leg_right_abad[2] = 0x00;
    ret2.payload.leg_right_abad[3] = 0x00;
    ret2.payload.leg_right_abad[4] = 0x00;
    ret2.payload.leg_right_abad[5] = 0x00;
    ret2.payload.leg_right_abad[6] = 0x00;
    ret2.payload.leg_right_abad[7] = 0x00;

    ret2.payload.leg_right_hip[0] = 0xFF;
    ret2.payload.leg_right_hip[1] = 0xFF;
    ret2.payload.leg_right_hip[2] = 0xFF;
    ret2.payload.leg_right_hip[3] = 0xFF;
    ret2.payload.leg_right_hip[4] = 0xFF;
    ret2.payload.leg_right_hip[5] = 0xFF;
    ret2.payload.leg_right_hip[6] = 0xFF;
    ret2.payload.leg_right_hip[7] = 0xFF;

    ret2.payload.leg_right_knee[0] = 0x7F;
    ret2.payload.leg_right_knee[1] = 0xFF;
    ret2.payload.leg_right_knee[2] = 0x7F;
    ret2.payload.leg_right_knee[3] = 0xF0;
    ret2.payload.leg_right_knee[4] = 0x00;
    ret2.payload.leg_right_knee[5] = 0x00;
    ret2.payload.leg_right_knee[6] = 0x07;
    ret2.payload.leg_right_knee[7] = 0xFF;
    // 3号腿给越界值
    cmd.q_abad[3] = 0;
    cmd.q_hip[3] = -100;
    cmd.q_knee[3] = 200;

    cmd.qd_abad[3] = 0;
    cmd.qd_hip[3] = -50;
    cmd.qd_knee[3] = 100;

    cmd.t_abad[3] = -20;
    cmd.t_hip[3] = 0;
    cmd.t_knee[3] = 20;

    cmd.kp_abad[3] = -100;
    cmd.kp_hip[3] = 0;
    cmd.kp_knee[3] = 800;

    cmd.kd_abad[3] = -5;
    cmd.kd_hip[3] = 6;
    cmd.kd_knee[3] = 0;

    ret2.payload.leg_left_abad[0] = 0x7F;
    ret2.payload.leg_left_abad[1] = 0xFF;
    ret2.payload.leg_left_abad[2] = 0x7F;
    ret2.payload.leg_left_abad[3] = 0xF0;
    ret2.payload.leg_left_abad[4] = 0x00;
    ret2.payload.leg_left_abad[5] = 0x00;
    ret2.payload.leg_left_abad[6] = 0x00;
    ret2.payload.leg_left_abad[7] = 0x00;

    ret2.payload.leg_left_hip[0] = 0x00;
    ret2.payload.leg_left_hip[1] = 0x00;
    ret2.payload.leg_left_hip[2] = 0x00;
    ret2.payload.leg_left_hip[3] = 0x00;
    ret2.payload.leg_left_hip[4] = 0x00;
    ret2.payload.leg_left_hip[5] = 0xFF;
    ret2.payload.leg_left_hip[6] = 0xF7;
    ret2.payload.leg_left_hip[7] = 0xFF;

    ret2.payload.leg_left_knee[0] = 0xFF;
    ret2.payload.leg_left_knee[1] = 0xFF;
    ret2.payload.leg_left_knee[2] = 0xFF;
    ret2.payload.leg_left_knee[3] = 0xFF;
    ret2.payload.leg_left_knee[4] = 0xFF;
    ret2.payload.leg_left_knee[5] = 0x00;
    ret2.payload.leg_left_knee[6] = 0x0F;
    ret2.payload.leg_left_knee[7] = 0xFF;
    // 一次性测试
    motorProtocol->Send(cmd);
    EXPECT_TRUE(Spi2CanIsSame(sendBuf[0], ret1));
    EXPECT_TRUE(Spi2CanIsSame(sendBuf[1], ret2));
}

TEST_F(TestMotorProtocol, MotorProtocolMit_Recv)
{
    SubCfg_QrMotor motorCfg;
    motorCfg.motorModel.fill(MotorModel::haitai);
    motorCfg.chiral.fill(false);
    auto motorProtocol = make_unique<MotorProtocolMit>(motorCfg);
    recvBuf[0].payload.leg_left_abad[0] = 1;
    recvBuf[0].payload.leg_left_abad[1] = 0x80;
    recvBuf[0].payload.leg_left_abad[2] = 0x9C;
    recvBuf[0].payload.leg_left_abad[3] = 0x80;
    recvBuf[0].payload.leg_left_abad[4] = 0xB9;
    recvBuf[0].payload.leg_left_abad[5] = 0x4;
    recvBuf[0].payload.leg_left_abad[6] = 0x0;
    recvBuf[0].payload.leg_left_abad[7] = 0x0;

    recvBuf[0].payload.leg_left_hip[0] = 2;
    recvBuf[0].payload.leg_left_hip[1] = 0x80;
    recvBuf[0].payload.leg_left_hip[2] = 0x51;
    recvBuf[0].payload.leg_left_hip[3] = 0x7F;
    recvBuf[0].payload.leg_left_hip[4] = 0xF7;
    recvBuf[0].payload.leg_left_hip[5] = 0xDD;
    recvBuf[0].payload.leg_left_hip[6] = 0x0;
    recvBuf[0].payload.leg_left_hip[7] = 0x0;

    recvBuf[0].payload.leg_left_knee[0] = 3;
    recvBuf[0].payload.leg_left_knee[1] = 0x7E;
    recvBuf[0].payload.leg_left_knee[2] = 0xB2;
    recvBuf[0].payload.leg_left_knee[3] = 0x7E;
    recvBuf[0].payload.leg_left_knee[4] = 0xE5;
    recvBuf[0].payload.leg_left_knee[5] = 0xC5;
    recvBuf[0].payload.leg_left_knee[6] = 0x0;
    recvBuf[0].payload.leg_left_knee[7] = 0x0;

    auto data = motorProtocol->Recv();
    msg::qr::motor_ret ret;
    ret.q_abad[1] = 0.456115;
    ret.q_hip[1] = 0.237530;
    ret.q_knee[1] = -0.971977;
    ret.qd_abad[1] = 0.252747;
    ret.qd_hip[1] = -0.010989;
    ret.qd_knee[1] = -0.384615;
    ret.t_abad[1] = 2.290110;
    ret.t_hip[1] = -0.303297;
    ret.t_knee[1] = -5.015385;

    EXPECT_NEAR(data.q_abad[1], ret.q_abad[1], 0.0001);
    EXPECT_NEAR(data.q_hip[1], ret.q_hip[1], 0.0001);
    EXPECT_NEAR(data.q_knee[1], ret.q_knee[1], 0.0001);

    EXPECT_NEAR(data.qd_abad[1], ret.qd_abad[1], 0.0001);
    EXPECT_NEAR(data.qd_hip[1], ret.qd_hip[1], 0.0001);
    EXPECT_NEAR(data.qd_knee[1], ret.qd_knee[1], 0.0001);

    EXPECT_NEAR(data.t_abad[1], ret.t_abad[1], 0.0001);
    EXPECT_NEAR(data.t_hip[1], ret.t_hip[1], 0.0001);
    EXPECT_NEAR(data.t_knee[1], ret.t_knee[1], 0.0001);
}

/* 海泰电机部分情况下，会上报超出理论范围的异常新值，但不影响结果
 * 如下是测的几组异常新值和对应的同位置的正常值如下
 * 异常新值     同位置正常值
 *   -0.5639     0.4852
 *   -0.5931     0.4532
 *   -0.5406     0.5056
 *   0.5464      -0.4998
 *   0.5639      -0.4823
 *   0.5523      -0.4969
 *   0.5348      -0.5144
 *   -0.5260     0.5202
 *   -0.5377     0.5086
 *   -0.5698     0.4765
 */
TEST_F(TestMotorProtocol, CalOverflow)
{
    SubCfg_QrMotor motorCfg;
    motorCfg.motorModel.fill(MotorModel::haitai);
    motorCfg.chiral.fill(false);
    auto motorProtocol = make_unique<MotorProtocolMit>(motorCfg);

    double bias = 0;
    double overFlow = M_PI / 6;  // 溢出值
    double initV = 0.4716;       // 初始值

    double test = -0.5639;
    auto ret = motorProtocol->CalOverflow(initV, test, overFlow);
    int minPos = std::min_element(ret.begin(), ret.end()) - ret.begin();
    if (minPos == 0) {
        bias = initV + 2 * overFlow;
    } else if (minPos == 1) {
        bias = initV - 2 * overFlow;
    } else {
        bias = initV;
    }
    // cout << "test1 " << test - bias << " " << bias << endl;
    EXPECT_NEAR(test - bias, 0.0117, 0.001);

    test = -0.5931;
    ret = motorProtocol->CalOverflow(initV, test, overFlow);
    minPos = std::min_element(ret.begin(), ret.end()) - ret.begin();
    if (minPos == 0) {
        bias = initV + 2 * overFlow;
    } else if (minPos == 1) {
        bias = initV - 2 * overFlow;
    } else {
        bias = initV;
    }
    // cout << "test2 " << test - bias << " " << bias << endl;
    EXPECT_NEAR(test - bias, -0.0175, 0.001);

    test = 0.5464;
    ret = motorProtocol->CalOverflow(initV, test, overFlow);
    minPos = std::min_element(ret.begin(), ret.end()) - ret.begin();
    if (minPos == 0) {
        bias = initV + 2 * overFlow;
    } else if (minPos == 1) {
        bias = initV - 2 * overFlow;
    } else {
        bias = initV;
    }
    // cout << "test3 " << test - bias << " " << bias << endl;
    EXPECT_NEAR(test - bias, 0.0748, 0.001);

    test = 0.5639;
    ret = motorProtocol->CalOverflow(initV, test, overFlow);
    minPos = std::min_element(ret.begin(), ret.end()) - ret.begin();
    if (minPos == 0) {
        bias = initV + 2 * overFlow;
    } else if (minPos == 1) {
        bias = initV - 2 * overFlow;
    } else {
        bias = initV;
    }
    // cout << "test4 " << test - bias << " " << bias << endl;
    EXPECT_NEAR(test - bias, 0.0923, 0.001);

    test = -0.5756;
    ret = motorProtocol->CalOverflow(initV, test, overFlow);
    minPos = std::min_element(ret.begin(), ret.end()) - ret.begin();
    if (minPos == 0) {
        bias = initV + 2 * overFlow;
    } else if (minPos == 1) {
        bias = initV - 2 * overFlow;
    } else {
        bias = initV;
    }
    // cout << "test5 " << test - bias << " " << bias << endl;
    EXPECT_NEAR(test - bias, 0.0, 0.001);

    // 正常值，正方向
    test = 0.4888;
    ret = motorProtocol->CalOverflow(initV, test, overFlow);
    minPos = std::min_element(ret.begin(), ret.end()) - ret.begin();
    if (minPos == 0) {
        bias = initV + 2 * overFlow;
    } else if (minPos == 1) {
        bias = initV - 2 * overFlow;
    } else {
        bias = initV;
    }
    // cout << "test5 " << test - bias << " " << bias << endl;
    EXPECT_NEAR(test - bias, 0.0172, 0.001);

    // 正常值，负方向
    test = 0.4517;
    ret = motorProtocol->CalOverflow(initV, test, overFlow);
    minPos = std::min_element(ret.begin(), ret.end()) - ret.begin();
    if (minPos == 0) {
        bias = initV + 2 * overFlow;
    } else if (minPos == 1) {
        bias = initV - 2 * overFlow;
    } else {
        bias = initV;
    }
    // cout << "test5 " << test - bias << " " << bias << endl;
    EXPECT_NEAR(test - bias, -0.0199, 0.001);

    // 正常值，正方向溢出
    test = -0.4888;
    ret = motorProtocol->CalOverflow(initV, test, overFlow);
    minPos = std::min_element(ret.begin(), ret.end()) - ret.begin();
    if (minPos == 0) {
        bias = initV + 2 * overFlow;
    } else if (minPos == 1) {
        bias = initV - 2 * overFlow;
    } else {
        bias = initV;
    }
    // cout << "test5 " << test - bias << " " << bias << endl;
    EXPECT_NEAR(test - bias, 0.0868, 0.001);
}
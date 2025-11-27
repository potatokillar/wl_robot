
#pragma once

#include "cppType.hpp"

static constexpr uint8_t ARM_MOTOR_SIZE = 7;

namespace msg
{

struct mit_motor_cmd
{
    double alpha{0};
    double torq{0};
    double blta{0};
    double k2{0};
    double k1{0};
};

struct mit_motor_ret
{
    double alpha{0};
    double torq{0};
    double blta{0};
    uint8_t sta{0};
};

struct imu_data
{
    double gyro[3]{0};
    double acc[3]{0};
    double ang[3]{0};
    double quat[4]{0};
};

// 调试关心的数据
struct watch_data
{
    double a[3]{0};
    double b[3]{0};
    double c[3]{0};

    double ra[3]{0};
    double rb[3]{0};
    double rc[3]{0};
};

struct watch_data_a
{
    double a[3]{0};
    double ra[3]{0};
};
struct watch_data_b
{
    double b[3]{0};
    double rb[3]{0};
};
struct watch_data_c
{
    double c[3]{0};
    double rc[3]{0};
};

struct arm_cmd
{
    struct MitMotorCmd
    {
        double alpha{0};
        double torq{0};
        double blta{0};
        double k2{0};
        double k1{0};
    } motor[ARM_MOTOR_SIZE];
};

struct arm_data
{
    struct MitMotorData
    {
        double alpha{0};
        double torq{0};
        double blta{0};
        uint8_t sta{0};
    } motor[ARM_MOTOR_SIZE];
};

struct arm_teach
{
    double x;
    double y;
    double z;
    double Rx;
    double Ry;
    double Rz;
};

enum class arm_motor_ctrl
{
    enable,
    disable,
};

enum class arm_motor_state
{
    enable,
    disable,
    error,
};

struct arm_motor_info
{
    std::map<std::string, std::bitset<32>> ioInSta;
    std::map<std::string, std::bitset<32>> ioOutSta;

    arm_motor_state state{arm_motor_state::disable};
    std::array<u16, ARM_MOTOR_SIZE> errCode;
};

namespace wheel
{
struct motor_cmd
{
    msg::mit_motor_cmd leg[4][4];
};

struct motor_ret
{
    msg::mit_motor_ret leg[4][4];
};

};  // namespace wheel

namespace qr
{
struct motor_cmd
{
    msg::mit_motor_cmd leg[4][3];
};

struct motor_ret
{
    msg::mit_motor_ret leg[4][3];
};
};  // namespace qr

namespace human
{
struct motor_cmd65
{
    msg::mit_motor_cmd arm[2][6];
    msg::mit_motor_cmd leg[2][5];
};

struct motor_ret65
{
    msg::mit_motor_ret arm[2][6];
    msg::mit_motor_ret leg[2][5];
};
};  // namespace human

namespace bs
{
struct gamepad_cmd
{
    int8_t btnA{0};
    int8_t btnB{0};
    int8_t btnX{0};
    int8_t btnY{0};
    int8_t btnLb{0};
    int8_t btnRb{0};
    int8_t btnBack{0};
    int8_t btnStart{0};
    int8_t btnLpress{0};
    int8_t btnRpress{0};
    int8_t btnUp{0};
    int8_t btnDown{0};
    int8_t btnLeft{0};
    int8_t btnRight{0};

    double stickLx{0};
    double stickLy{0};
    double stickRx{0};
    double stickRy{0};
    double stickLt{0};
    double stickRt{0};
};

};  // namespace bs

namespace net
{

};  // namespace net
};  // namespace msg
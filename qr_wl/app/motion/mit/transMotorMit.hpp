
#pragma once
#include "baseline.hpp"
#include "deviceType.hpp"
#include "mitMotorType.hpp"
#include "spi2can.hpp"
#include "spi2canV2.hpp"
#include "spi2canV3.hpp"

struct MitMotorCmd : public msg::mit_motor_cmd
{
    u16 id;
};

struct MitMotorRet : public msg::mit_motor_ret
{
    u8 t;
    u8 err;
    u16 id;
};

class TransformMitMotor
{
public:
    TransformMitMotor(MotorModel model);

    bool CmdToCan(uint8_t *msg, double p, double v, double t, double kpp, double kdd);
    void CmdToCanV2(u8 *msg, const msg::mit_motor_cmd &cmd, u16 id);
    CanData CmdToCanV3(const msg::mit_motor_cmd &cmd, u16 id);

    void CanToCmd(uint8_t *msg, double *p, double *v, double *t, double *kpp, double *kdd);

    bool CanToData(const uint8_t *msg, double *p, double *v, double *t, u8 *T, u8 *err);
    bool CanToData(const uint8_t *msg, double *p, double *v, double *t);

    bool CanToDataV2(const u8 *msg, msg::mit_motor_ret *ret, u16 *id, u8 *T, u8 *err);
    bool CanToDataV2(const u8 *msg, msg::mit_motor_ret *ret, u16 *id);
    bool CanToDataV2(const u8 *msg, msg::mit_motor_ret *ret);

    bool CanToDataV3(const CanData &msg, msg::mit_motor_ret *ret, u16 *id, u8 *T, u8 *err);
    bool CanToDataV3(const CanData &msg, msg::mit_motor_ret *ret, u16 *id);
    bool CanToDataV3(const CanData &msg, msg::mit_motor_ret *ret);

    std::optional<u16> GetId(const u8 *msg);

private:
    /// Returns maximum of x, y ///
    double fmaxf1(double x, double y) { return (((x) > (y)) ? (x) : (y)); }
    /// Returns minimum of x, y ///
    double fminf1(double x, double y) { return (((x) < (y)) ? (x) : (y)); }
    /// Returns maximum of x, y, z ///
    double fmaxf3(double x, double y, double z) { return (x > y ? (x > z ? x : z) : (y > z ? y : z)); }
    /// Returns minimum of x, y, z ///
    double fminf3(double x, double y, double z) { return (x < y ? (x < z ? x : z) : (y < z ? y : z)); }
    /// Scales the lenght of vector (x, y) to be <= limit ///
    void limit_norm(double *x, double *y, double limit);
    /// Converts a double to an unsigned int, given range and number of bits ///
    int float_to_uint(double x, double x_min, double x_max, int bits);
    /// converts unsigned int to double, given range and number of bits ///
    double uint_to_float(int x_int, double x_min, double x_max, int bits);

private:
    double P_MIN = -12.5f;  // Radians
    double P_MAX = 12.5f;
    double V_MIN = -8.0f;  // Rad/s
    double V_MAX = 8.0f;
    double T_MIN = -144.0f;
    double T_MAX = 144.0f;
    double KP_MIN = 0.0f;  // N-m/rad
    double KP_MAX = 500.0f;
    double KD_MIN = 0.0f;  // N-m/rad/s
    double KD_MAX = 5.0f;
};

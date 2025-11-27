
#include "transMotorMit.hpp"

using namespace std;

TransformMitMotor::TransformMitMotor(MotorModel model)
{
    switch (model) {
        case MotorModel::haitai:
            P_MIN = -100;
            P_MAX = 100;
            V_MIN = -45;
            V_MAX = 45;
            T_MIN = -18;
            T_MAX = 18;
            break;
        case MotorModel::tMotor_10_9:
            P_MIN = -12.5;
            P_MAX = 12.5;
            V_MIN = -50;
            V_MAX = 50;
            T_MIN = -65;
            T_MAX = 65;
            break;
        case MotorModel::tMotor_12_8:
            P_MIN = -12.5;
            P_MAX = 12.5;
            V_MIN = -15.7;
            V_MAX = 15.7;
            T_MIN = -70;
            T_MAX = 70;
            break;
        case MotorModel::tMotor_80_64:
            P_MIN = -12.5;
            P_MAX = 12.5;
            V_MIN = -8;
            V_MAX = 8;
            T_MIN = -144;
            T_MAX = 144;
            KP_MIN = 0;
            KP_MAX = 500;
            KD_MIN = 0;
            KD_MAX = 50;
            break;
        case MotorModel::gim10015_9:
            P_MIN = -14;
            P_MAX = 14;
            V_MIN = -17;
            V_MAX = 17;
            T_MIN = -70;
            T_MAX = 70;
            KP_MIN = 0;
            KP_MAX = 500;
            KD_MIN = 0;
            KD_MAX = 25;
            break;
        default:
            break;
    }
}

bool TransformMitMotor::CmdToCan(uint8_t *msg, double p, double v, double t, double kpp, double kdd)
{
    double p_des = fminf1(fmaxf1(P_MIN, p), P_MAX);
    double v_des = fminf1(fmaxf1(V_MIN, v), V_MAX);
    double kp = fminf1(fmaxf1(KP_MIN, kpp), KP_MAX);
    double k1 = fminf1(fmaxf1(KD_MIN, kdd), KD_MAX);
    double t_ff = fminf1(fmaxf1(T_MIN, t), T_MAX);

    uint16_t p_int = float_to_uint(p_des, P_MIN, P_MAX, 16);
    uint16_t v_int = float_to_uint(v_des, V_MIN, V_MAX, 12);
    uint16_t kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12);
    uint16_t kd_int = float_to_uint(k1, KD_MIN, KD_MAX, 12);
    uint16_t t_int = float_to_uint(t_ff, T_MIN, T_MAX, 12);

    msg[0] = p_int >> 8;
    msg[1] = p_int & 0xFF;
    msg[2] = v_int >> 4;
    msg[3] = ((v_int & 0xF) << 4) | (kp_int >> 8);
    msg[4] = kp_int & 0xFF;
    msg[5] = kd_int >> 4;
    msg[6] = ((kd_int & 0xF) << 4) | (t_int >> 8);
    msg[7] = t_int & 0xff;
    return true;
}

void TransformMitMotor::CmdToCanV2(u8 *msg, const msg::mit_motor_cmd &cmd, u16 id)
{
    double p_des = fminf1(fmaxf1(P_MIN, cmd.alpha), P_MAX);
    double v_des = fminf1(fmaxf1(V_MIN, cmd.torq), V_MAX);
    double kp = fminf1(fmaxf1(KP_MIN, cmd.k2), KP_MAX);
    double k1 = fminf1(fmaxf1(KD_MIN, cmd.k1), KD_MAX);
    double t_ff = fminf1(fmaxf1(T_MIN, cmd.blta), T_MAX);

    uint16_t p_int = float_to_uint(p_des, P_MIN, P_MAX, 16);
    uint16_t v_int = float_to_uint(v_des, V_MIN, V_MAX, 12);
    uint16_t kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12);
    uint16_t kd_int = float_to_uint(k1, KD_MIN, KD_MAX, 12);
    uint16_t t_int = float_to_uint(t_ff, T_MIN, T_MAX, 12);

    msg[0] = id;
    msg[1] = id >> 8;
    msg[2] = p_int >> 8;
    msg[3] = p_int & 0xFF;
    msg[4] = v_int >> 4;
    msg[5] = ((v_int & 0xF) << 4) | (kp_int >> 8);
    msg[6] = kp_int & 0xFF;
    msg[7] = kd_int >> 4;
    msg[8] = ((kd_int & 0xF) << 4) | (t_int >> 8);
    msg[9] = t_int & 0xff;
}

CanData TransformMitMotor::CmdToCanV3(const msg::mit_motor_cmd &cmd, u16 id)
{
    double p_des = fminf1(fmaxf1(P_MIN, cmd.alpha), P_MAX);
    double v_des = fminf1(fmaxf1(V_MIN, cmd.torq), V_MAX);
    double kp = fminf1(fmaxf1(KP_MIN, cmd.k2), KP_MAX);
    double k1 = fminf1(fmaxf1(KD_MIN, cmd.k1), KD_MAX);
    double t_ff = fminf1(fmaxf1(T_MIN, cmd.blta), T_MAX);

    uint16_t p_int = float_to_uint(p_des, P_MIN, P_MAX, 16);
    uint16_t v_int = float_to_uint(v_des, V_MIN, V_MAX, 12);
    uint16_t kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12);
    uint16_t kd_int = float_to_uint(k1, KD_MIN, KD_MAX, 12);
    uint16_t t_int = float_to_uint(t_ff, T_MIN, T_MAX, 12);

    CanData ret;
    ret.id = id;
    ret.data[0] = p_int >> 8;
    ret.data[1] = p_int & 0xFF;
    ret.data[2] = v_int >> 4;
    ret.data[3] = ((v_int & 0xF) << 4) | (kp_int >> 8);
    ret.data[4] = kp_int & 0xFF;
    ret.data[5] = kd_int >> 4;
    ret.data[6] = ((kd_int & 0xF) << 4) | (t_int >> 8);
    ret.data[7] = t_int & 0xff;
    return ret;
}

void TransformMitMotor::CanToCmd(uint8_t *msg, double *p, double *v, double *t, double *kpp, double *kdd)
{
    int p_int = (msg[0] << 8) | msg[1];
    int v_int = (msg[2] << 4) | (msg[3] >> 4);
    int kp_int = ((msg[3] & 0xF) << 8) | msg[4];
    int kd_int = (msg[5] << 4) | (msg[6] >> 4);
    int t_int = ((msg[6] & 0xF) << 8) | msg[7];

    *p = uint_to_float(p_int, P_MIN, P_MAX, 16);
    *v = uint_to_float(v_int, V_MIN, V_MAX, 12);
    *t = uint_to_float(kp_int, KP_MIN, KP_MAX, 12);
    *kpp = uint_to_float(kd_int, KD_MIN, KD_MAX, 12);
    *kdd = uint_to_float(t_int, T_MIN, T_MAX, 12);
}

bool TransformMitMotor::CanToData(const uint8_t *msg, double *p, double *v, double *t, u8 *T, u8 *err)
{
    uint16_t id = msg[0];
    uint16_t p_int = (msg[1] << 8) | msg[2];
    uint16_t v_int = (msg[3] << 4) | (msg[4] >> 4);
    uint16_t i_int = ((msg[4] & 0xF) << 8) | msg[5];
    *T = msg[6];
    *err = msg[7];

    *p = uint_to_float(p_int, P_MIN, P_MAX, 16);
    *v = uint_to_float(v_int, V_MIN, V_MAX, 12);
    *t = uint_to_float(i_int, -T_MAX, T_MAX, 12);
    return true;
}
bool TransformMitMotor::CanToData(const uint8_t *msg, double *p, double *v, double *t)
{
    u8 T = 0;
    u8 err = 0;
    return CanToData(msg, p, v, t, &T, &err);
}

bool TransformMitMotor::CanToDataV2(const u8 *msg, msg::mit_motor_ret *ret, u16 *id, u8 *T, u8 *err)
{
    if ((msg[1] & 0x80) == 0x80) {
        return false;
    }

    *id = (u16)msg[2];

    uint16_t p_int = (msg[3] << 8) | msg[4];
    uint16_t v_int = (msg[5] << 4) | (msg[6] >> 4);
    uint16_t i_int = ((msg[6] & 0xF) << 8) | msg[7];
    *T = msg[8];
    *err = msg[9];

    ret->alpha = uint_to_float(p_int, P_MIN, P_MAX, 16);
    ret->torq = uint_to_float(v_int, V_MIN, V_MAX, 12);
    ret->blta = uint_to_float(i_int, -T_MAX, T_MAX, 12);

    return true;
}
bool TransformMitMotor::CanToDataV2(const u8 *msg, msg::mit_motor_ret *ret, u16 *id)
{
    u8 t, err;
    return CanToDataV2(msg, ret, id, &t, &err);
}
bool TransformMitMotor::CanToDataV2(const u8 *msg, msg::mit_motor_ret *ret)
{
    u8 t, err;
    u16 id;
    return CanToDataV2(msg, ret, &id, &t, &err);
}

bool TransformMitMotor::CanToDataV3(const CanData &msg, msg::mit_motor_ret *ret, u16 *id, u8 *T, u8 *err)
{
    *id = msg.id;
    if (msg.id & 0x8000) {
        // if (access("/tmp/motor_right_all.enable", F_OK)){
        //     LOG_INFO("id: {:X} msg.data[0]: {:X}", *id, msg.data[0]);
        //     printf("msg: ");
        //     for (int i = 0; i < 8; ++i) {
        //         printf("%02X ", msg.data[i]);
        //     }
        //     printf("\n");
        //     return true;
        // }
        return false;
    }
    if (*id != msg.data[0]) {
        LOG_WARN("mit id not match {:X} {:X}", *id, msg.data[0]);  // 只警告
    }
    uint16_t p_int = (msg.data[1] << 8) | msg.data[2];
    uint16_t v_int = (msg.data[3] << 4) | (msg.data[4] >> 4);
    uint16_t i_int = ((msg.data[4] & 0xF) << 8) | msg.data[5];
    *T = msg.data[6];
    *err = msg.data[7];

    ret->alpha = uint_to_float(p_int, P_MIN, P_MAX, 16);
    ret->torq = uint_to_float(v_int, V_MIN, V_MAX, 12);
    ret->blta = uint_to_float(i_int, -T_MAX, T_MAX, 12);

    return true;
}
bool TransformMitMotor::CanToDataV3(const CanData &msg, msg::mit_motor_ret *ret, u16 *id)
{
    u8 t, err;
    return CanToDataV3(msg, ret, id, &t, &err);
}
bool TransformMitMotor::CanToDataV3(const CanData &msg, msg::mit_motor_ret *ret)
{
    u8 t, err;
    u16 id;
    return CanToDataV3(msg, ret, &id, &t, &err);
}

std::optional<u16> TransformMitMotor::GetId(const u8 *msg)
{
    if ((msg[0] & 0x80) == 0x80) {
        return std::nullopt;
    }
    return (msg[0] << 8) | msg[1];
}

void TransformMitMotor::limit_norm(double *x, double *y, double limit)
{
    double norm = sqrt(*x * *x + *y * *y);
    if (norm > limit) {
        *x = *x * limit / norm;
        *y = *y * limit / norm;
    }
}

int TransformMitMotor::float_to_uint(double x, double x_min, double x_max, int bits)
{
    double span = x_max - x_min;
    double offset = x_min;
    return (int)((x - offset) * ((double)((1 << bits) - 1)) / span);
}

double TransformMitMotor::uint_to_float(int x_int, double x_min, double x_max, int bits)
{
    double span = x_max - x_min;
    double offset = x_min;
    return ((double)x_int) * span / ((double)((1 << bits) - 1)) + offset;
}

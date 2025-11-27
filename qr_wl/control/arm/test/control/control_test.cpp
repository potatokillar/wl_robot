#include "control_test.hpp"

#include "armParam.hpp"

using namespace ::std;

double CtrlTest::rpm2ang(double rpm)
{
    double ang = 0.0;
    ang = rpm * 6;
    return ang;
}

double CtrlTest::ang2rad(double ang)
{
    double rad = 0.0;

    rad = ang * M_PI / 180;

    return rad;
}

double CtrlTest::rad2ang(double rad)
{
    double ang = 0.0;
    ang = rad * 180 / M_PI;
    return ang;
}

double CtrlTest::rpm2rad(double rpm)
{
    double rad = 0.0;
    rad = rpm * 2 * M_PI / 60;
    return rad;
}

Vec4<double> CtrlTest::rpy2quat(Vec3<double> rpy)
{
    Vec4<double> quat;

    quat(3) = cos(rpy(0) / 2) * cos(rpy(1) / 2) * cos(rpy(2) / 2) + sin(rpy(0) / 2) * sin(rpy(1) / 2) * sin(rpy(2) / 2);
    quat(0) = sin(rpy(0) / 2) * cos(rpy(1) / 2) * cos(rpy(2) / 2) - cos(rpy(0) / 2) * sin(rpy(1) / 2) * sin(rpy(2) / 2);
    quat(1) = cos(rpy(0) / 2) * sin(rpy(1) / 2) * cos(rpy(2) / 2) + sin(rpy(0) / 2) * cos(rpy(1) / 2) * sin(rpy(2) / 2);
    quat(2) = cos(rpy(0) / 2) * cos(rpy(1) / 2) * sin(rpy(2) / 2) - sin(rpy(0) / 2) * sin(rpy(1) / 2) * cos(rpy(2) / 2);

    return quat;
}

Vec3<double> CtrlTest::quat2rpy(Vec4<double> quat)
{
    Vec3<double> rpy;
    rpy(0) = atan2(2 * (quat(3) * quat(0) + quat(1) * quat(2)), 1 - 2 * (pow(quat(0), 2) + pow(quat(1), 2)));
    rpy(1) = asin(2 * (quat(3) * quat(1) - quat(0) * quat(2)));
    rpy(2) = atan2(2 * (quat(3) * quat(2) + quat(0) * quat(1)), 1 - 2 * (pow(quat(1), 2) + pow(quat(2), 2)));

    return rpy;
}

Vec3<double> CtrlTest::rotM2rpy(Mat4<double> rotM)
{
    Vec3<double> rpy;

    if (fabs(rotM(2, 2)) < 0.000001 && fabs(rotM(1, 2)) < 0.00001) {
        rpy(0) = 0;
        rpy(1) = 0.5 * M_PI;
        rpy(2) = atan2(rotM(3, 0), rotM(3, 3));
    } else {
        rpy(0) = atan2(-rotM(1, 2), rotM(2, 2));
        rpy(1) = atan2(rotM(0, 2), cos(rpy(0)) * rotM(2, 2) - sin(rpy(0)) * rotM(1, 2));
        rpy(2) = atan2(-rotM(0, 3), rotM(0, 0));
    }
    return rpy;
}

Mat3<double> CtrlTest::rpy2rotM(Vec3<double> rpy)
{
    Mat3<double> rotM;

    rotM << cos(rpy(1)) * cos(rpy(2)), cos(rpy(2)) * sin(rpy(0)) * sin(rpy(1)) - cos(rpy(0)) * sin(rpy(2)), sin(rpy(0)) * sin(rpy(2)) + cos(rpy(1)) * cos(rpy(2)) * sin(rpy(1)), cos(rpy(1)) * sin(rpy(2)), cos(rpy(0)) * cos(rpy(2)) + sin(rpy(0)) * sin(rpy(1)) * sin(rpy(2)), cos(rpy(0)) * sin(rpy(1)) * sin(rpy(2)) - cos(rpy(2)) * sin(rpy(0)), -sin(rpy(1)), cos(rpy(1)) * sin(rpy(0)), cos(rpy(0)) * cos(rpy(1));

    return rotM;
}

Mat4<double> CtrlTest::RPY2ROTM(Vec6<double> RPY)
{
    Mat4<double> ROTM;

    ROTM(0, 0) = cos(RPY(4)) * cos(RPY(5));
    ROTM(0, 1) = cos(RPY(5)) * sin(RPY(3)) * sin(RPY(4)) - cos(RPY(3)) * sin(RPY(5));
    ROTM(0, 2) = sin(RPY(3)) * sin(RPY(5)) + cos(RPY(3)) * cos(RPY(5)) * sin(RPY(4));
    ROTM(0, 3) = RPY(0);
    ROTM(1, 0) = cos(RPY(4)) * sin(RPY(5));
    ROTM(1, 1) = cos(RPY(3)) * cos(RPY(5)) + sin(RPY(3)) * sin(RPY(4)) * sin(RPY(5));
    ROTM(1, 2) = cos(RPY(3)) * sin(RPY(4)) * sin(RPY(5)) - cos(RPY(5)) * sin(RPY(3));
    ROTM(1, 3) = RPY(1);
    ROTM(2, 0) = -sin(RPY(4));
    ROTM(2, 1) = cos(RPY(4)) * sin(RPY(3));
    ROTM(2, 2) = cos(RPY(3)) * cos(RPY(4));
    ROTM(2, 3) = RPY(2);
    ROTM(3, 0) = 0.0;
    ROTM(3, 1) = 0.0;
    ROTM(3, 2) = 0.0;
    ROTM(3, 3) = 1.0;

    return ROTM;
}

double CtrlTest::Five_plan(double S, double Plan_t, double t)
{
    double V_start, V_end, acc_start, acc_end, a4, a5, a6, Y = 0, state = 0;
    V_start = 0;
    V_end = 0;
    acc_start = 0;
    acc_end = 0;

    a4 = (20 * S - (8 * V_end + 12 * V_start) * Plan_t - (3 * acc_start - acc_end) * std::pow(Plan_t, 2)) / (2 * std::pow(Plan_t, 3));
    a5 = (-30 * S + (14 * V_end + 16 * V_start) * Plan_t + (3 * acc_start - 2 * acc_end) * std::pow(Plan_t, 2)) / (2 * std::pow(Plan_t, 4));
    a6 = (12 * S - 6 * (V_end + V_start) * Plan_t + (acc_end - acc_start) * std::pow(Plan_t, 2)) / (2 * std::pow(Plan_t, 5));
    state = 1;

    if (state == 1) {
        if (t <= Plan_t) {
            Y = a4 * std::pow(t, 3) + a5 * std::pow(t, 4) + a6 * std::pow(t, 5);
        } else {
            Y = a4 * std::pow(Plan_t, 3) + a5 * std::pow(Plan_t, 4) + a6 * std::pow(Plan_t, 5);
        }
    } else {
        Y = 0;
        state = 0;
    }
    return Y;
}

Mat4<double> CtrlTest::Fkine(Vec6<double> Radian)
{
    auto& armParam = GetArmParam();

    double d1, d4, dt, a2, a3;
    Mat4<double> Tbase, Ttool, T1, T2, T3, T4, T5, T6, KPS44;
    double s1 = 0.0, s2, s3, s4, s5, s6, c1, c2, c3, c4, c5, c6;

    d1 = armParam.kine_param.d(0);
    d4 = armParam.kine_param.d(3);
    dt = armParam.kine_param.d(5);
    a2 = armParam.kine_param.a(1);
    a3 = armParam.kine_param.a(2);

    s1 = sin(Radian(0));
    s2 = sin(Radian(1));
    s3 = sin(Radian(2));
    s4 = sin(Radian(3));
    s5 = sin(Radian(4));
    s6 = sin(Radian(5));
    c1 = cos(Radian(0));
    c2 = cos(Radian(1));
    c3 = cos(Radian(2));
    c4 = cos(Radian(3));
    c5 = cos(Radian(4));
    c6 = cos(Radian(5));

    Tbase << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, d1, 0, 0, 0, 1;
    Ttool << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, dt, 0, 0, 0, 1;

    T1 << c1, -s1, 0, 0, s1, c1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
    T2 << c2, -s2, 0, 0, 0, 0, 1, 0, -s2, -c2, 0, 0, 0, 0, 0, 1;
    T3 << c3, -s3, 0, a2, s3, c3, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
    T4 << c4, -s4, 0, a3, 0, 0, 1, d4, -s4, -c4, 0, 0, 0, 0, 0, 1;
    T5 << c5, -s5, 0, 0, 0, 0, -1, 0, s5, c5, 0, 0, 0, 0, 0, 1;
    T6 << c6, -s6, 0, 0, 1, 0, -s6, -c6, 0, 0, 0, 0, 0, 1;

    KPS44 = Tbase * T1 * T2 * T3 * T4 * T5 * T6 * Ttool;

    return KPS44;
}

Vec6<double> CtrlTest::Ikine(Mat4<double> KPS44, Vec6<double> q_reward)
{
    auto& armParam = GetArmParam();

    double s1, s3, s4, s5, s23, c1, c3, c4, c5, c23, K = 0, ss = 0, zyj = 8, sss1 = 0, sss2 = 0, ssss1 = 0, ssss2 = 0, sssss1 = 0, sssss2 = 0, h = 0, Min_ = 0;
    Mat4<double> T_base, T_tool, inv_Tbase, inv_Ttool, R44;
    Vec2<double> Q1;
    Vec4<double> Q3;
    Mat2<double> Q2, Q23, Q4, Q5, Q6;
    Mat86<double> QQ;

    double d1, d4, dt, a2, a3, offset2;
    Vec6<double> Qmax, Qmin, new_q;
    Vec8<double> bz, Min;

    bz << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    Min << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    d1 = armParam.kine_param.d(0);
    d4 = armParam.kine_param.d(3);
    dt = armParam.kine_param.d(5);
    a2 = armParam.kine_param.a(1);
    a3 = armParam.kine_param.a(2);
    offset2 = armParam.kine_param.offset(1);
    Qmax = armParam.axis.A_max;
    Qmin = armParam.axis.A_min;

    Q1.fill(0);
    Q2.fill(0);
    Q3.fill(0);
    Q4.fill(0);
    Q5.fill(0);
    Q6.fill(0);
    Q23.fill(0);
    QQ.fill(0);

    T_base << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, d1, 0, 0, 0, 1;
    T_tool << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, dt, 0, 0, 0, 1;
    inv_Tbase = T_base.inverse();
    inv_Ttool = T_tool.inverse();

    R44 = inv_Tbase * KPS44 * inv_Ttool;

    q_reward(1) = q_reward(1) + offset2;

    Q1(0) = atan2(R44(1, 3), R44(0, 3));
    Q1(1) = atan2(-R44(1, 3), -R44(0, 3));

    K = (pow(R44(0, 3), 2) + pow(R44(1, 3), 2) + pow(R44(2, 3), 2) - pow(a2, 2) - pow(a3, 2) - pow(d4, 2)) / (2 * a2);
    if (pow(a3, 2) + pow(d4, 2) - pow(K, 2) >= 0) {
        ss = sqrt(pow(a3, 2) + pow(d4, 2) - pow(K, 2));
        Q3(0) = atan2(a3, d4) - atan2(K, ss);
        Q3(1) = atan2(a3, d4) - atan2(K, -ss);
    } else {
        Q3(0) = 10000;
        Q3(1) = 10000;
    }

    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
            if (i == 1) {
                Q3(2) = Q3(0);
                Q3(3) = Q3(1);
            }
            s1 = sin(Q1(i));
            s3 = sin(Q3(j));
            c1 = cos(Q1(i));
            c3 = cos(Q3(j));
            Q23(i, j) = atan2((-a3 - a2 * c3) * R44(2, 3) - (c1 * R44(0, 3) + s1 * R44(1, 3)) * (d4 - a2 * s3), (a2 * s3 - d4) * R44(2, 3) + (a3 + a2 * c3) * (c1 * R44(0, 3) + s1 * R44(1, 3)));
            Q2(i, j) = Q23(i, j) - Q3(j);
            s23 = sin(Q23(i, j));
            c23 = cos(Q23(i, j));
            sss1 = -R44(0, 2) * s1 + R44(1, 2) * c1;
            sss2 = -R44(0, 2) * c1 * c23 - R44(1, 2) * s1 * c23 + R44(2, 2) * s23;

            if (fabs(sss1) < 0.000000001 && fabs(sss2) < 0.000000001) {
                Q4(i, j) = q_reward[3];
                Q5(i, j) = 0;
                s4 = sin(Q4(i, j));
                c4 = cos(Q4(i, j));
                s5 = sin(Q5(i, j));
                c5 = cos(Q5(i, j));
                sssss1 = -R44(0, 0) * (c1 * c23 * s4 - s1 * c4) - R44(1, 0) * (s1 * c23 * s4 + c1 * c4) + R44(2, 0) * (s23 * s4);
                sssss2 = R44(0, 0) * ((c1 * c23 * c4 + s1 * s4) * c5 - c1 * s23 * s5) + R44(1, 0) * ((s1 * c23 * c4 - c1 * s4) * c5 - s1 * s23 * s5) - R44(2, 0) * (s23 * c4 * c5 + c23 * s5);
                Q6(i, j) = atan2(sssss1, sssss2);
            } else {
                Q4(i, j) = atan2(sss1, sss2);
                s4 = sin(Q4(i, j));
                c4 = cos(Q4(i, j));

                ssss1 = -R44(0, 2) * (c1 * c23 * c4 + s1 * s4) - R44(1, 2) * (s1 * c23 * c4 - c1 * s4) + R44(2, 2) * (s23 * c4);
                ssss2 = R44(0, 2) * (-c1 * s23) + R44(1, 2) * (-s1 * s23) + R44(2, 2) * (-c23);
                Q5(i, j) = atan2(ssss1, ssss2);  // right
                s5 = sin(Q5(i, j));
                c5 = cos(Q5(i, j));
                sssss1 = -R44(0, 0) * (c1 * c23 * s4 - s1 * c4) - R44(1, 0) * (s1 * c23 * s4 + c1 * c4) + R44(2, 0) * (s23 * s4);
                sssss2 = R44(0, 0) * ((c1 * c23 * c4 + s1 * s4) * c5 - c1 * s23 * s5) + R44(1, 0) * ((s1 * c23 * c4 - c1 * s4) * c5 - s1 * s23 * s5) - R44(2, 0) * (s23 * c4 * c5 + c23 * s5);
                Q6(i, j) = atan2(sssss1, sssss2);
            }

            QQ(h, 0) = Q1(i);
            QQ(h, 1) = Q2(i, j) - offset2;
            QQ(h, 2) = Q3(j);
            QQ(h, 3) = Q4(i, j);
            QQ(h, 4) = Q5(i, j);
            QQ(h, 5) = Q6(i, j);
            QQ(h + 4, 0) = Q1(i);
            QQ(h + 4, 1) = Q2(i, j) - offset2;
            QQ(h + 4, 2) = Q3(j);
            QQ(h + 4, 3) = Q4(i, j) + M_PI;
            QQ(h + 4, 4) = -Q5(i, j);
            QQ(h + 4, 5) = Q6(i, j) + M_PI;
            h = h + 1;
        }
    }
    for (int m = 0; m < 8; m++) {
        for (int n = 0; n < 6; n++) {
            if (QQ(m, n) < Qmin(n)) {
                QQ(m, n) = QQ(m, n) + 2 * M_PI;
            }
            if (QQ(m, n) > Qmax(n)) {
                QQ(m, n) = QQ(m, n) - 2 * M_PI;
            }
            if (fabs(QQ(m, n) - q_reward(n)) > M_PI) {
                if (QQ(m, n) >= 0) {
                    QQ(m, n) = QQ(m, n) - 2 * M_PI;
                } else {
                    QQ(m, n) = QQ(m, n) + 2 * M_PI;
                }
            }
        }
    }
    //     search_angle
    for (int i = 0; i < 8; i++) {
        if ((QQ(i, 0) >= Qmin(0)) && (QQ(i, 0 <= Qmax(0))) && (QQ(i, 1) >= Qmin(1)) && (QQ(i, 1) <= Qmax(1)) && (QQ(i, 2) >= Qmin(2)) && (QQ(i, 2) <= Qmax(2)) && (QQ(i, 2) >= Qmin(3)) && (QQ(i, 3) <= Qmax(3)) && (QQ(i, 4) >= Qmin(4)) && (QQ(i, 4) <= Qmax(4)) && (QQ(i, 5) >= Qmin(5)) && (QQ(i, 5) <= Qmax(5))) {
            bz(i) = 1;                                                                                                                                                                                         // ���н��־���?1�Ǳ�ʾ�ǿ��н�
            Min(i) = fabs(q_reward(0) - QQ(i, 0)) + fabs(q_reward(1) - QQ(i, 1)) + fabs(q_reward(2) - QQ(i, 2)) + fabs(q_reward(3) - QQ(i, 3)) + fabs(q_reward(4) - QQ(i, 4)) + fabs(q_reward(5) - QQ(i, 5));  // �����Ӧ���ֵ
        } else {
            bz(i) = 0;
            Min(i) = 0;
        }
    }

    Min_ = Min(0) + Min(1) + Min(2) + Min(3) + Min(4) + Min(5) + Min(6) + Min(7);
    for (int i = 0; i < 8; i++) {
        if (bz(i) == 1 && Min_ >= Min(i)) {
            Min_ = Min(i);
            zyj = i;
        }
    }
    if (zyj == 8)  // best
    {
        for (int i = 0; i < 6; i++) {
            new_q[i] = q_reward(i);
        }
    } else {
        for (int i = 0; i < 6; i++) {
            new_q[i] = QQ(zyj, i);
        }
    }

    return new_q;
}

#pragma once
#include <set>

#include "baseline.hpp"
class CtrlTest
{
public:
    CtrlTest();

    double rpm2ang(double rpm);
    double rpm2rad(double rpm);
    double rad2ang(double rad);
    double ang2rad(double ang);

    Vec4<double> rpy2quat(Vec3<double> rpy);
    Vec3<double> quat2rpy(Vec4<double> quat);
    Vec3<double> rotM2rpy(Mat4<double> rotM);
    Mat3<double> rpy2rotM(Vec3<double> rpy);
    Mat4<double> RPY2ROTM(Vec6<double> RPY);

    double Five_plan(double S, double Plan_t, double t);

    Mat4<double> Fkine(Vec6<double> Radian);
    Vec6<double> Ikine(Mat4<double> KPS44, Vec6<double> rerad);

private:
};
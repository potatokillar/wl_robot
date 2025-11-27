
#pragma once
#include "baseline.hpp"

struct ImuData
{
    // 1. 姿态角结算时所使用的坐标系为东北天坐标系，正方向放置模块,
    // 如下图所示向左 为 X 轴，向前为 Y 轴，向上为 Z 轴,参考IMU图示。
    // 欧拉角表示姿态时的坐标系旋转顺序定义为为 z-y-x,即先绕 z 轴转，再绕 y 轴转，再绕 x 轴转。
    // 2. 滚转角的范围虽然是±180 度，但实际上由于坐标旋转顺序是 Z-Y-X，在表示姿态 的时候，俯仰角(Y 轴)的范围只有±90 度，
    // 超过 90 度后会变换到小于 90 度，同时 让X轴的角度大于180度。详细原理请大家自行百度欧拉角及姿态表示的相关信息。
    // 3. 由于三轴是耦合的，只有在小角度的时候会表现出独立变化，在大角度的时候姿态 角度会耦合变化，
    // 比如当 Y 轴接近 90 度时，即使姿态只绕 Y 轴转动，X 轴的角度 也会跟着发生较大变化，这是欧拉角表示姿态的固有问题。
    Vec3<double> rpy;
    Vec3<double> w;
    Vec3<double> a;
    Vec4<double> quat;
    ImuData() { setZero(); }
    void setZero()
    {
        rpy.setZero();
        w.setZero();
        a.setZero();
        quat << 1.0, 0.0, 0.0, 0.0;
    }
    bool LoadData(const msg::imu_data& data)  // 载入数据,由基础软件来源(sim/real)
    {
        for (int i = 0; i < 3; i++) {
            rpy(i) = data.ang[i];
            w(i) = data.gyro[i];
            a(i) = data.acc[i];
            quat(i) = data.quat[i];
        }
        quat(3) = data.quat[3];
        return true;
    }

    // 实测，只要实现了复合运算符，其对应的双目运算符也被实现了。不知道是官方库还是三方库的功劳
    ImuData& operator+=(const ImuData& data)
    {
        rpy += data.rpy;
        w += data.w;
        a += data.a;
        return *this;
    }

    ImuData& operator-=(const ImuData& data)
    {
        rpy -= data.rpy;
        w -= data.w;
        a -= data.a;
        return *this;
    }

    ImuData& operator/=(int size)
    {
        rpy /= size;
        w /= size;
        a /= size;
        return *this;
    }
};

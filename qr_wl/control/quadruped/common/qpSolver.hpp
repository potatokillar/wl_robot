
#ifndef QPSOLVER_H
#define QPSOLVER_H

#include <mutex>
#include <thread>

#include "baseline.hpp"
#include "quadrupedParam.hpp"
#include "simpleSemaphore.h"

struct QpInput
{
    bool holdFlagValue = false;

    Vec4<bool> contactFlagVec;
    Mat34<double> pFootMatrix;
    Vec3<double> aComVec;
    Vec3<double> alphaComVec;
    Mat3<double> bodyIworld;
    double bodyMass = 0.0;
    QpInput()
    {
        contactFlagVec.fill(false);
        pFootMatrix.setZero();
        aComVec.setZero();
        alphaComVec.setZero();
        bodyIworld.setZero();
    }
};

struct QpOutput
{
    // double forceFootMatrix[3][4];
    Mat34<double> forceFootMatrix;
    QpOutput()
    {
        forceFootMatrix.setZero();
        // 防止第一次动力学求解不及时，导致的踉跄
        // 被注释的代码是表示站立时候应该的默认qp值，因本类中是无法知道状态的
        // 所以qp结果的初始化固定为walk状态的初始值。
        // 此时只有两条腿接触
        // forceFootMatrix[2][0] = GetParam().qr.DOG_M * G(2) / -4.0;
        // forceFootMatrix[2][1] = GetParam().qr.DOG_M * G(2) / -4.0;
        // forceFootMatrix[2][2] = GetParam().qr.DOG_M * G(2) / -4.0;
        // forceFootMatrix[2][3] = GetParam().qr.DOG_M * G(2) / -4.0;

        // forceFootMatrix[2][0] = 0.0;
        // forceFootMatrix[2][1] = GetParam().qr.DOG_M * G(2) / -2.0;
        // forceFootMatrix[2][2] = GetParam().qr.DOG_M * G(2) / -2.0;
        // forceFootMatrix[2][3] = 0.0;

        forceFootMatrix(2, 0) = 0.0;
        forceFootMatrix(2, 1) = GetQrParam().DOG_M * G(2) / -2.0;
        forceFootMatrix(2, 2) = GetQrParam().DOG_M * G(2) / -2.0;
        forceFootMatrix(2, 3) = 0.0;
    }
};

class QPSolver
{
public:
    QPSolver();
    virtual ~QPSolver();
    void LoadData(const QpInput &in);
    const QpOutput &GetResult();
    void Loop();

private:
    void runSolver();
    void GetInputPara();
    void SetOutputResult();

private:
    Mat34<double> pFoot;
    Vec3<double> aCom;
    Vec3<double> alphaCom;
    Vec4<bool> contactFlag;
    bool holdFlag;
    Mat3<double> bodyIworld_;
    double bodyMass_ = 0.0;
    Mat34<double> forceFoot;
    Vec12<double> qpResultOld;

    SimpleSemaphore _qpSemaphore;  // 实例化
    QpInput input_;
    QpOutput output_, outputOld_;
    std::thread thread_;
    bool running_;
    mutable std::timed_mutex mutex_;
    const QuadrupedParam &qrParam = GetQrParam();
};

#endif
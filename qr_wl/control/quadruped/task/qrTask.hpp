
#pragma once
#include <thread>

#include "baseline.hpp"
#include "contrType.hpp"
#include "innerType.hpp"
#include "mathTools.hpp"
class QrTask
{
public:
    QrTask();
    RetState mixedDance0();
    RetState mixedDance1();
    int handShake(Vec3<double> angleIn, double periodIn, double showTimeIn);
    int handShake2();
    int pushUp(int pushNum);
    int rpyDance();
    int takeBow(Vec6<double> angleIn, Vec6<double> periodIn, double showTimeIn);
    RetState singleDance0();
    RetState singleDance1();
    RetState singleDance2();
    RetState singleDance3();
    RetState singleDance4();
    int takeBow2();
    int happyJump(DVec<double> angleVec, DVec<double> xVec, DVec<double> yVec, DVec<double> zVecDown, DVec<double> zVecUp, DVec<double> delayVec, double kBackIn);
    int moonWalk(int timesIn, double stepLxIn, double zDownIn, double zUpIn, double delayIn, double kBackIn);

    int moonWalk2(Mat34<double> pFootInA, Mat34<double> pFootInB, int timesIn, double zDownIn, double zUpIn, double delayIn, double kBackIn);
    int longJump(Vec4<Mat34<double>> kpIn, Vec4<Mat34<double>> kdIn, Vec4<Mat34<double>> kfIn, double angleIn, double xIn, double yIn, double zDownIn, double zUpIn, double delayIn,
                 double kBackIn);
    int longJump2(Vec4<Mat34<double>> kpIn, Vec4<Mat34<double>> kdIn, Vec4<Mat34<double>> kfIn, double angleIn, double xIn, double yIn, double zDownIn, double zUpIn,
                  double delayIn, double kBackIn);

private:
    // 为了生成pFoot
    Vec3<double> pCom;
    Mat34<double> pRoll, pHip, pFoot;
    double LEG_L0 = 0.08;
    double LEG_L1 = 0.213;
    double LEG_L2 = 0.213;
    double BODY_Lx = 0.38;  // from real Go1 robot
    double BODY_Ly = 0.093;
    double DOG_M = 5.56 + (0.68 + 1.01 + 0.2) * 4;
    double bodyHeight = sqrt(pow(LEG_L1, 2) + pow(LEG_L2, 2));
    double G = 9.8;

    MathFun mathFun_;

    Mat34<double> tmpForceFoot;
    // 命令序列指针，数据量较大，基于性能的考虑，这里采用指针。
    // 这个指针最终会传递到最底层，生命周期大于本类
    // todo 在本类存入数据，接收方清除数据，目前是通过消息阻塞机制来保证数据塞入和清除不冲突的
    std::shared_ptr<std::vector<CmdVal2>> cmdQ_;
    RetState SendCmd(int len);
};
#include "qpSolver.hpp"

#include <math.h>

#include <iostream>
#include <qpOASES.hpp>

#include "baseline.hpp"

using namespace std;
using namespace Eigen;
using namespace qpOASES;

QPSolver::QPSolver()
{
    // QP求解器的变量也需要重置，否则出现不确定情况异常
    holdFlag = false;
    pFoot.setZero();
    forceFoot.setZero();
    aCom.setZero();
    alphaCom.setZero();
    contactFlag.setZero();
    qpResultOld.setZero();

    _qpSemaphore.init(0);  // 信号量
    running_ = true;
    thread_ = std::thread(&QPSolver::Loop, this);
}

QPSolver::~QPSolver()
{
    running_ = false;
    if (thread_.joinable()) {
        thread_.join();
    }
}

/**
 * @description: 载入qp求解需要的数据
 * @param &in
 * @return {}
 */
void QPSolver::LoadData(const QpInput &in)
{
    std::unique_lock lock(mutex_, TimerTools::ToUs(100));
    if (lock.owns_lock()) {
        input_ = in;
        _qpSemaphore.increment();  // 信号量
    }
}

/**
 * @description: 获取结果，若此时数据未计算完毕，则返回上一次的结果
 * @return {}
 */
const QpOutput &QPSolver::GetResult()
{
    std::unique_lock lock(mutex_, TimerTools::ToUs(100));
    if (lock.owns_lock()) {
        outputOld_ = output_;
        return output_;
    } else {
        return outputOld_;
    }
}

/**
 * @description: 取出输入的参数，
 * @return {}
 */
void QPSolver::GetInputPara()
{
    std::unique_lock lock(mutex_);
    holdFlag = input_.holdFlagValue;
    contactFlag = input_.contactFlagVec;
    pFoot = input_.pFootMatrix;
    aCom = input_.aComVec;
    alphaCom = input_.alphaComVec;
    bodyIworld_ = input_.bodyIworld;
    bodyMass_ = input_.bodyMass;
}

/**
 * @description: 将计算结果填入内部结构体中 供输出使用
 * @return {}
 */
void QPSolver::SetOutputResult()
{
    std::unique_lock lock(mutex_);
    output_.forceFootMatrix = forceFoot;
}

void QPSolver::Loop()
{
    while (running_) {
        // 2ms等待，因算法线程2ms一次，因此每2ms必定会有数据到来
        if (_qpSemaphore.decrementTimeout(0, 2000000) == true) {
            GetInputPara();
            runSolver();
            SetOutputResult();
        }
    }
}

void QPSolver::runSolver()
{
    // double t1 = TimerTools::GetNowTickUs();
    Mat3<double> I_3;
    I_3.setIdentity();

    if (holdFlag == true)  // 如果用户速度比较小就不响应
    {
        contactFlag.fill(true);
    }

    DMat<double> COF_A = DMat<double>::Zero(6, 4 * 3);
    DMat<double> pFootContact = DMat<double>::Zero(3, 4);
    DMat<double> COF_B = DMat<double>::Zero(6, 1);
    DMat<double> COF_W = DMat<double>::Zero(4 * 3, 4 * 3);
    DMat<double> COF_U = DMat<double>::Zero(4 * 3, 4 * 3);
    DMat<double> COF_H = DMat<double>::Zero(4 * 3, 4 * 3);
    DMat<double> COF_g = DMat<double>::Zero(4 * 3, 1);
    DMat<double> lb = DMat<double>::Zero(4 * 3, 1);
    DMat<double> ub = DMat<double>::Zero(4 * 3, 1);

    pFootContact = pFoot;  // 这pFoot来源于外部，应当使用与世界坐标系姿态相同的坐标表达，也就是RPY=0时的pFoot
    // cout << "pFootContact =  " << pFootContact << endl;

    for (int i = 0; i < 4; i++) {
        COF_A.block(0, i * 3, 3, 3) << I_3;  // 提取块大小为(3,3),起始于(0,i*3),往里面填单位矩阵
        COF_A.block(3, i * 3, 3, 3) << 0, -pFootContact(2, i), pFootContact(1, i), pFootContact(2, i), 0, -pFootContact(0, i), -pFootContact(1, i), pFootContact(0, i),
            0;  // 生成叉乘矩阵,提取块大小为(p,q),起始于(i,j),往里面填
    }

    // 加负号是因为求出来的是期望反作用力，我要的是足部力
    COF_B << bodyMass_ * -aCom, bodyIworld_ * -alphaCom;  // aCom里边包含g，与alphaCom，都应当表达在rpy=0的世界系下

    // COF_W.setIdentity();  // W的前三个系数代表线加速度，后三个代表角加速度的权重
    Vec12<double> w, u;
    w << 10, 10, 4, 10, 10, 4, 10, 10, 4, 10, 10, 4;
    u << 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3;
    COF_W = w.asDiagonal();
    COF_U = u.asDiagonal();
    // cout << "A = " << COF_A << endl;
    // cout << "B = " << COF_B << endl;

    COF_H = 2 * (COF_A.transpose() * qrParam.matrixS * COF_A + qrParam.kW * COF_W + qrParam.kU * COF_U);  // 代价函数取极小可以整体乘以系数不影响求解结果
    // COF_H = 2 * (COF_A.transpose() * matrixS * COF_A);  //查看VBL那篇论文，去掉这一项,推导见有道笔记
    //  COF_H = ((COF_H.transpose() + COF_H) / 2).eval();
    //  COF_g = -2 * (COF_B.transpose() * matrixS * COF_A);   //1 * k不对 应该是k*1，但里面的值是相等的，相差一个转置
    // 注意：宇树书本P115页结果是-bT*S*A  与
    COF_g = -2 * (COF_A.transpose() * qrParam.matrixS * COF_B + qrParam.kU * COF_U * qpResultOld);  // 20220712发现错误，来源于Matlab与QP库标准表达式不同, k * 1
    // cout << "H = " << COF_H << endl;

    // Ax.setIdentity();     //lbA <= A * x <= ubA      A metrix

    for (int i = 0; i < 4; i++)  // lbA   ubA metrix
    {
        //  最大边界
        if (contactFlag(i)) {
            ub.block(i * 3, 0, 3, 1) << qrParam.mu * qrParam.fZmax, qrParam.mu * qrParam.fZmax,
                -10.0;  // 提取块大小为(3,1),起始于(i*3,0),往里面填,这里的边界条件改善对上楼梯有帮助, 0.7代表二分之根号二
            // 最小边界
            lb.block(i * 3, 0, 3, 1) << -qrParam.mu * qrParam.fZmax, -qrParam.mu * qrParam.fZmax,
                -qrParam.fZmax;  // 提取块大小为(3,1),起始于(i*3,0),往里面填
        } else {
            ub.block(i * 3, 0, 3, 1) << 0.0, 0.0, 0.0;
            lb.block(i * 3, 0, 3, 1) << 0.0, 0.0, 0.0;
        }
    }

    // USING_NAMESPACE_QPOASES
    real_t qpCOF_H[4 * 3 * 4 * 3];
    real_t qpCOF_g[4 * 3];

    real_t qpub[4 * 3];
    real_t qplb[4 * 3];

    for (int i = 0; i < 4 * 3; i++) {
        for (int j = 0; j < 4 * 3; j++) {
            *(qpCOF_H + i * 4 * 3 + j) = COF_H(i, j);
            //*(qpAx + i * temp_contactNum * 3 + j) = Ax(i, j);
        }
    }

    for (int i = 0; i < 4 * 3; i++) {
        *(qpCOF_g + i) = COF_g(i);
        //*(qpubA + i) = ubA(i);
        //*(qplbA + i) = lbA(i);
        *(qpub + i) = ub(i);
        *(qplb + i) = lb(i);
    }

    // QProblem qpSolver(temp_contactNum * 3, temp_contactNum * 3);
    QProblemB qpSolver(4 * 3);  // problem B

    Options options;
    // options.setToMPC();            //设置为MPC模式，求解速度优先，可以打开源码查看其它模式
    options.printLevel = PL_NONE;  // None   Low(if error)
    qpSolver.setOptions(options);

    int_t nWSR_my = 200;      // 最大迭代次数
    real_t cpu_time = 0.002;  // 最大CPU运行时间，可能导致求解不精确  求解时间大多数在200us以内，有些会到500-1300us左右，但也够了

    qpSolver.init(qpCOF_H, qpCOF_g, qplb, qpub, nWSR_my, &cpu_time);  // problem B  add cpu time

    real_t qpResult[4 * 3];
    qpSolver.getPrimalSolution(qpResult);

    forceFoot.setZero();  // 必须要重置，否则上一次的残留值会影响,这个力也是世界坐标系下的力，rpy=0时
    for (int i = 0; i < 4; i++) {
        forceFoot(0, i) = qpResult[0 + i * 3];
        forceFoot(1, i) = qpResult[1 + i * 3];
        forceFoot(2, i) = qpResult[2 + i * 3];

        qpResultOld(i * 3, 0) = qpResult[0 + i * 3];
        qpResultOld(i * 3 + 1, 0) = qpResult[1 + i * 3];
        qpResultOld(i * 3 + 2, 0) = qpResult[2 + i * 3];
    }
    /*
    double t2 = TimerTools::GetNowTickUs();
    if (t2 - t1 > 500) {
        cout << "dt= " << t2 - t1 << "us" << endl;
    }
    */
    // cout << "temp_contactNum = " << temp_contactNum << endl;
}
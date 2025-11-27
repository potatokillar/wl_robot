
#pragma once
#include "baseline.hpp"
#include "quadrupedParam.hpp"

struct GaitPara
{
    double tStance = 0.2;
    double tSwing = tStance;
    // double stepPeriod = tStance + tSwing;
    double stepZ = 0.2;
    // Vec4<double> tDelta;
    Vec3<double> kpRPY;
    Vec3<double> kdRPY;
    GaitPara()
    {
        // tDelta << 0.0, tStance, tStance, 0.0;
        kpRPY.setZero();
        kdRPY.setZero();
    }
};

// 当前的参数
struct CurPara
{
    Vec3<double> rpy;
    Mat3<double> R;
    Mat3<double> Ryaw0;
    Vec3<double> w;
    Vec3<double> a;
    Vec3<double> alpha;
    Vec3<double> v;
    Vec4<double> quat;
    double h = GetQrParam().bodyHeight;
    double stepZ = 0;
    WalkMode mode = WalkMode::aiNormal;  // 当前步行模式
    GaitType gait = GaitType::tort;      // 当前步态
    CurPara()
    {
        R.setIdentity();
        rpy.setZero();
        w.setZero();
        a.setZero();
        alpha.setZero();
        v.setZero();
        quat.setZero();
    }
};

// 期望的参数
struct DesPara
{
    Vec3<double> rpy;
    Mat3<double> R;
    Mat3<double> Ryaw0;
    Vec3<double> a;
    Vec3<double> alpha;
    Vec3<double> v;
    Vec3<double> w;
    // bool holdFlag = false;
    class
    {
    private:
        bool val = false;
        u64 time = TimerTools::GetNowTickMs();

    public:
        void Set(bool v)
        {
            if ((v == true) && (TimerTools::GetNowTickMs() - time > 1000)) {
                val = v;
                time = TimerTools::GetNowTickMs();
            } else if (v == false) {
                val = v;
                //  time = TimerTools::GetNowTickMs();
            }
        }
        bool Get() { return val; }
        void UpdateTrueTime() { time = TimerTools::GetNowTickMs(); }
    } holdflag;
    double stepZ = 0;
    double h = GetQrParam().bodyHeight;
    WalkMode mode = WalkMode::aiNormal;  // 步行模式
    GaitType gait = GaitType::tort;      // 步态

    DesPara()
    {
        R.setIdentity();
        a.setZero();
        alpha.setZero();
        v.setZero();
        w.setZero();
        rpy.setZero();
    }
};

// 状态切换时间，可能扩展参数
struct StateShiftTime
{
    double T = 1.0;  // 总需要时间t
    double t = 0.0;  // 当前时间ti
    StateShiftTime(std::initializer_list<double> Tin)
    {
        // 无参数时，默认1.0
        if (Tin.size() > 0) {
            T = *Tin.begin();
        } else {
            T = 1.0;
        }
    }
};

struct StateBackFlipData
{
    StateShiftTime backFlipTime{2.5};
    DataCmd motorCmdBackFlip;
    void Reset()
    {
        backFlipTime.t = 0;
        motorCmdBackFlip = {};
    }
};

// JP算法求出的数据，公开给其它模块
struct JpDataOutput
{
    // 用于基于概率的触底检测新算法
    Vec4<int> sPhi;
    Vec4<double> phi;

    // 仅用于算法内部
    Vec3<double> aDesire;    // 控制算法期望的加速度
    Vec4<bool> contactFlag;  // 控制算法期望的接触状态

    double desireHeight;  // 期望高度，来自desire
    bool holdFlag = false;

    JpDataOutput()
    {
        sPhi.fill(0);
        phi.fill(0.0f);
        aDesire.setZero();
        contactFlag.fill(false);
        desireHeight = GetQrParam().bodyHeight;
    }
};

// 算法内部数据，作用域为jp模块和状态机
struct JpData
{
    CurPara current;    // 当前数据
    DesPara desire;     // 期望数据，通过用户指令和上一个期望指令生成
                        // ControllerCmd userCtrl;  // 用户控制指令
    GaitPara gaitPara;  // 步态参数

    Mat34<double> pFoot;  // foot位置0 1 2 3；
    Mat34<double> pFootPeriodEnd;
    Vec4<bool> contactFlag;  // 按照时间决定的接触状态，每2ms重置

    Vec4<int> sPhi;    // 描述某个腿是摆动相或支撑相，生成参考轨迹有用
    Vec4<double> phi;  // 描述某个腿处于摆动相或支撑相的比例，但四个腿本身的值是完全相同的
    bool fromSim_{false};
    bool noMotor{false};
    Mat34<double> pFootForStand;  // 记录站立状态时的pfoot，数据在多个状态中会更新
    // Mat34<double> pFootForHold;   // 记录刚切换hold时的pFoot，用于随姿态角更新pFoot
    // Mat34<double> qFall;  // 记录摔倒时刻返回的关节脚，供恢复时使用
    bool endPeriodFlag = true;
    Mat34<double> tauCmdRecord;
    Mat34<double> qCmdRecord;
    JpDataOutput output;

    bool fallFlag = false;

    JpData()
    {
        pFoot = GetQrParam().pFoot;
        pFootPeriodEnd = GetQrParam().pFoot;
        contactFlag.fill(true);
        sPhi.fill(0);
        phi.fill(0.0f);
        pFootForStand = GetQrParam().pFoot;
        tauCmdRecord.setZero();
        qCmdRecord.setZero();
    }
};

// 四足启动参数，注意和全局启动参数的区别
struct QrBootArgs
{
    double dt;            // 核心模块的调度周期
    AlgoModel algoModel;  // 算法模型
};

#pragma once
#include <memory>

#include "baseline.hpp"
#include "innerType.hpp"

// 默认的pid策略
class PidStrategy
{
public:
    // 默认pid策略是，直接返回min的值
    virtual Mat34<double> GetKp(const Mat34<double>& min, const Mat34<double>& max, const Mat34<double>& k);
    virtual Mat34<double> GetKd(const Mat34<double>& min, const Mat34<double>& max, const Mat34<double>& k);
    virtual Mat34<double> GetKf(const Mat34<double>& min, const Mat34<double>& max, const Mat34<double>& k);
};

// middle专用的pid策略
class PidStrategyMiddle : public PidStrategy
{
public:
    Mat34<double> GetKp(const Mat34<double>& min, const Mat34<double>& max, const Mat34<double>& k) override;
    Mat34<double> GetKd(const Mat34<double>& min, const Mat34<double>& max, const Mat34<double>& k) override;
    Mat34<double> GetKf(const Mat34<double>& min, const Mat34<double>& max, const Mat34<double>& k) override;
};

class PidStrategySwingLowKpKd : public PidStrategy
{
public:
    Mat34<double> GetKp(const Mat34<double>& min, const Mat34<double>& max, const Mat34<double>& k) override;
    Mat34<double> GetKd(const Mat34<double>& min, const Mat34<double>& max, const Mat34<double>& k) override;
    Mat34<double> GetKf(const Mat34<double>& min, const Mat34<double>& max, const Mat34<double>& k) override;
};

// 调整前馈力补偿
class TorqueModifyStrategy
{
public:
    virtual Mat34<double> GetTorque(const Mat34<double>& swingFootForce, Vec4<double> contactFlag);
};

class TorqueModifyMiddle : public TorqueModifyStrategy
{
public:
    Mat34<double> GetTorque(const Mat34<double>& swingFootForce, Vec4<double> contactFlag) override;
};

// 支持所有的策略
class StrategySelect
{
public:
    StrategySelect();
    // PID策略
    void SetPid(PidStrategyType type);
    Mat34<double> GetKp(const Mat34<double>& min, const Mat34<double>& max, const Mat34<double>& k) { return pid_->GetKp(min, max, k); }
    Mat34<double> GetKd(const Mat34<double>& min, const Mat34<double>& max, const Mat34<double>& k) { return pid_->GetKp(min, max, k); }
    Mat34<double> GetKf(const Mat34<double>& min, const Mat34<double>& max, const Mat34<double>& k) { return pid_->GetKp(min, max, k); }
    // 其他策略
    void SetTorqueModify(TorqueModifyStrategyType type);
    Mat34<double> GetTorque(const Mat34<double>& swingFootForce, Vec4<double> contactFlag) { return torqueModify_->GetTorque(swingFootForce, contactFlag); };

private:
    std::unique_ptr<PidStrategy> pid_;
    PidStrategyType pidType_;
    std::unique_ptr<TorqueModifyStrategy> torqueModify_;
    TorqueModifyStrategyType torqueModifyType_;
};
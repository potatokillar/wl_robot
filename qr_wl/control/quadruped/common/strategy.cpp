
#include "strategy.hpp"

Mat34<double> PidStrategy::GetKp(const Mat34<double>& min, const Mat34<double>& max, const Mat34<double>& k)
{
    (void)max;
    (void)k;
    return min;
}
Mat34<double> PidStrategy::GetKd(const Mat34<double>& min, const Mat34<double>& max, const Mat34<double>& k)
{
    (void)max;
    (void)k;
    return min;
}
Mat34<double> PidStrategy::GetKf(const Mat34<double>& min, const Mat34<double>& max, const Mat34<double>& k)
{
    (void)max;
    (void)k;
    return min;
}

/**
 * @description: middle策略
 * @return {}
 */
Mat34<double> PidStrategyMiddle::GetKp(const Mat34<double>& min, const Mat34<double>& max, const Mat34<double>& k)
{
    Mat34<double> I;
    I.fill(1);
    return min + (I - k).cwiseProduct(max - min);
}
Mat34<double> PidStrategyMiddle::GetKd(const Mat34<double>& min, const Mat34<double>& max, const Mat34<double>& k)
{
    Mat34<double> I;
    I.fill(1);
    return min + (I - k).cwiseProduct(max - min);
}
Mat34<double> PidStrategyMiddle::GetKf(const Mat34<double>& min, const Mat34<double>& max, const Mat34<double>& k) { return min + k.cwiseProduct(max - min); }

Mat34<double> PidStrategySwingLowKpKd::GetKp(const Mat34<double>& min, const Mat34<double>& max, const Mat34<double>& k) { return k + 0 * min + 0 * max; }
Mat34<double> PidStrategySwingLowKpKd::GetKd(const Mat34<double>& min, const Mat34<double>& max, const Mat34<double>& k) { return k + 0 * min + 0 * max; }
Mat34<double> PidStrategySwingLowKpKd::GetKf(const Mat34<double>& min, const Mat34<double>& max, const Mat34<double>& k) { return k + 0 * min + 0 * max; }

Mat34<double> TorqueModifyStrategy::GetTorque(const Mat34<double>& swingFootForce, Vec4<double> contactFlag)
{
    UNUSED(swingFootForce);
    UNUSED(contactFlag);
    Mat34<double> ret;
    ret.setZero();
    return ret;
}
Mat34<double> TorqueModifyMiddle::GetTorque(const Mat34<double>& swingFootForce, Vec4<double> contactFlag)
{
    Mat34<double> tmpF;
    for (int i = 0; i < 4; i++) {
        if (contactFlag(i) == 0) {
            // 只在Z轴添加前馈力，单腿质量
            tmpF.col(i) = swingFootForce.col(i);
        }
    }
    return tmpF;
}

/**
 * @description: 默认基类策略
 * @return {}
 */
StrategySelect::StrategySelect()
{
    pid_ = std::make_unique<PidStrategy>();
    pidType_ = PidStrategyType::base;

    torqueModify_ = std::make_unique<TorqueModifyStrategy>();
    torqueModifyType_ = TorqueModifyStrategyType::base;
}

/**
 * @description: 重新设置PID的策略
 * @param model
 * @return {}
 */
void StrategySelect::SetPid(PidStrategyType type)
{
    // 防止重复构造
    if (type == pidType_) {
        return;
    }
    if (type == PidStrategyType::middle) {
        pid_ = std::make_unique<PidStrategyMiddle>();
        pidType_ = type;
    } else if (type == PidStrategyType::swingLowKpKd) {
        pid_ = std::make_unique<PidStrategySwingLowKpKd>();
        pidType_ = type;
    }
}

void StrategySelect::SetTorqueModify(TorqueModifyStrategyType type)
{
    // 防止重复构造
    if (type == torqueModifyType_) {
        return;
    }
    if (type == TorqueModifyStrategyType::middle) {
        torqueModify_ = std::make_unique<TorqueModifyMiddle>();
        torqueModifyType_ = type;
    }
}
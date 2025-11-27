#include "Transformer.hpp"
/**
 * @description: MIT坐标系下关节角度转实物坐标系
 * @param {double} q被转换的关节角矩阵MIT坐标系下
 * @param {double} qInitMit MIT坐标系下初始关节角矩阵
 * @param {double} kneeRatio膝关节传动减速比
 * @return {Mat34<double>} 实物坐标系下的关节角矩阵
 */
Mat34<double> TransformCoordinate::qMit2real(const Mat34<double>& q, const Mat34<double>& qInitMit, double kneeRatio)
{
    Mat34<double> temp_q = q;  // MIT坐标系下的绝对位置命令，算法的输出是绝对位置

    temp_q -= qInitMit;  // MIT坐标系下的相对位置命令

    temp_q(1, 1) *= -1;
    temp_q(2, 1) *= -1;
    temp_q(0, 2) *= -1;
    temp_q(0, 3) *= -1;
    temp_q(1, 3) *= -1;
    temp_q(2, 3) *= -1;  // webot(sim)仿真下的相对位置命令，该坐标系初始位置与MIT相同，只是关节正方向为实物的右手定则

    // 在算法的real，始终以右手定则为电机的正方向
    // 实物电机的手性由电机驱动那边进行转换
    // 取消此处和其他类似地方的取反操作
    // temp_q = -temp_q;  //海泰电机实际为左手定则

    temp_q.row(2) *= kneeRatio;  // 转换到电机端需要更多关节角，考虑减速比

    return temp_q;
}
/**
 * @description: MIT坐标系下的关节角速度转实物坐标系
 * @param {double} qd MIT坐标系下的关节角速度矩阵
 * @param {double} kneeRatio膝关节减速比
 * @return {Mat34<double>} 实物坐标系下的关节角速度矩阵
 */
Mat34<double> TransformCoordinate::qdMit2real(const Mat34<double>& qd, double kneeRatio)
{
    Mat34<double> temp_qd = qd;  // MIT坐标系下的速度命令

    temp_qd(1, 1) *= -1;
    temp_qd(2, 1) *= -1;
    temp_qd(0, 2) *= -1;
    temp_qd(0, 3) *= -1;
    temp_qd(1, 3) *= -1;
    temp_qd(2, 3) *= -1;  // webot仿真下的速度命令

    // temp_qd = -temp_qd;  //海泰电机左手定则

    temp_qd.row(2) *= kneeRatio;  // 考虑减速比

    return temp_qd;
}
/**
 * @description: MIT坐标系下的力矩转实物坐标系下
 * @param {double} tau MIT坐标系下的力矩矩阵
 * @param {double} kneeRatio膝关节减速比
 * @return {Mat34<double>} 实物坐标系下的力矩矩阵
 */
Mat34<double> TransformCoordinate::tauMit2real(const Mat34<double>& tau, double kneeRatio)
{
    Mat34<double> temp_tau = tau;  // MIT坐标系下的力矩命令

    temp_tau(1, 1) *= -1;
    temp_tau(2, 1) *= -1;
    temp_tau(0, 2) *= -1;
    temp_tau(0, 3) *= -1;
    temp_tau(1, 3) *= -1;
    temp_tau(2, 3) *= -1;  // webot仿真下的力矩命令

    // temp_tau = -temp_tau;  //海泰电机左手定则

    temp_tau.row(2) /= kneeRatio;  // 考虑减速比
    return temp_tau;
}

//-------------------------------------------------------------------------------------------
/**
 * @description: 实物坐标系下的关节角转MIT坐标下
 * @param {double} qReal 实物坐标系下的关节角矩阵
 * @param {double} qInitMit MIT坐标系下的初始关节角矩阵
 * @param {double} kneeRatio膝关节减速比
 * @return {Mat34<double>} MIT坐标系下的关节角矩阵
 */
Mat34<double> TransformCoordinate::qReal2mit(const Mat34<double>& qReal, const Mat34<double>& qInitMit, double kneeRatio)
{
    Mat34<double> temp_q = qReal;
    temp_q.row(2) /= kneeRatio;
    //  temp_q = -temp_q;  // webot(sim)坐标系下的相对关节角命令

    temp_q(1, 1) *= -1;
    temp_q(2, 1) *= -1;
    temp_q(0, 2) *= -1;
    temp_q(0, 3) *= -1;
    temp_q(1, 3) *= -1;
    temp_q(2, 3) *= -1;  // MIT坐标系下的相对关节角命令

    temp_q += qInitMit;  // MIT坐标系下的绝对位置命令

    return temp_q;
}
/**
 * @description: 实物坐标系下的关节角速度转MIT坐标下
 * @param {double} qdReal 实物坐标系下的关节角速度矩阵
 * @param {double} kneeRatio膝关节减速比
 * @return {Mat34<double>} MIT坐标系下的关节角速度矩阵
 */
Mat34<double> TransformCoordinate::qdReal2mit(const Mat34<double>& qdReal, double kneeRatio)
{
    Mat34<double> temp_qd = qdReal;
    temp_qd.row(2) /= kneeRatio;  // 需要的是关节角速度 = 电机角速度 / 同步带减速比
                                  // temp_qd = -temp_qd;           // webot(sim)坐标系下的关节角速度

    temp_qd(1, 1) *= -1;
    temp_qd(2, 1) *= -1;
    temp_qd(0, 2) *= -1;
    temp_qd(0, 3) *= -1;
    temp_qd(1, 3) *= -1;
    temp_qd(2, 3) *= -1;  // MIT坐标系下的速度命令

    return temp_qd;
}
/**
 * @description: 实物坐标系下的关节力矩转MIT坐标下
 * @param {double} tauReal 实物坐标系下的关节力矩矩阵
 * @param {double} kneeRatio膝关节减速比
 * @return {Mat34<double>} MIT坐标系下的关节力矩矩阵
 */
Mat34<double> TransformCoordinate::tauReal2mit(const Mat34<double>& tauReal, double kneeRatio)
{
    Mat34<double> temp_tau = tauReal;
    temp_tau.row(2) *= kneeRatio;  // 需要的是关节的力矩 = 电机力矩 * 同步带减速比
                                   //   temp_tau = -temp_tau;          // webot坐标系下的力矩命令

    temp_tau(1, 1) *= -1;
    temp_tau(2, 1) *= -1;
    temp_tau(0, 2) *= -1;
    temp_tau(0, 3) *= -1;
    temp_tau(1, 3) *= -1;
    temp_tau(2, 3) *= -1;  // MIT坐标系下力矩命令

    return temp_tau;
}
/**
 * @description: 我的坐标系(DH法)下的关节角转MIT
 * @param {Mat34<double>} 我的坐标系下的关节角矩阵
 * @return {Mat34<double>} MIT坐标系下的关节角矩阵
 */
Mat34<double> TransformCoordinate::my2mit(const Mat34<double>& qmy)
{
    Mat34<double> qmit;
    qmit.setZero();
    qmit = qmy;
    for (int i = 0; i < 4; i++) {
        qmit(1, i) = qmy(1, i) - M_PI / 2.0;
    }
    return qmit;
}
/**
 * @description: MIT坐标系下的关节角转我的
 * @param {Mat34<double>} MIT坐标系下的关节角矩阵
 * @return {Mat34<double>} 我的坐标系下的关节角矩阵
 */
Mat34<double> TransformCoordinate::mit2my(const Mat34<double>& qmit)
{
    Mat34<double> qmy;
    qmy.setZero();
    qmy = qmit;
    for (int i = 0; i < 4; i++) {
        qmy(1, i) = qmit(1, i) + M_PI / 2.0;
    }
    return qmy;
}

/////////////////////////////////////////////////////////////////////////////////
/**
 * @description: MIT坐标系下关节角度转宇树坐标系
 * @param {double} q MIT坐标系下的关节角矩阵
 * @return {Mat34<double>} 宇树坐标系下的关节角矩阵
 */
Mat34<double> TransformCoordinate::qMit2unitree(const Mat34<double>& qmit)
{
    Mat34<double> temp_q = qmit;  // MIT坐标系下的绝对位置命令，算法的输出是绝对位置
    for (int i = 1; i < 3; i++) {
        for (int j = 0; j < 4; j++) {
            temp_q(i, j) = -qmit(i, j);
        }
    }
    return temp_q;
}
/**
 * @description: MIT坐标系下的关节角速度转宇树坐标系
 * @param {double} qd MIT坐标系下的关节角速度矩阵
 * @return {Mat34<double>} 宇树坐标系下的关节角速度矩阵
 */
Mat34<double> TransformCoordinate::qdMit2unitree(const Mat34<double>& qdmit)
{
    Mat34<double> temp_qd = qdmit;  // MIT坐标系下的速度命令
    for (int i = 1; i < 3; i++) {
        for (int j = 0; j < 4; j++) {
            temp_qd(i, j) = -qdmit(i, j);
        }
    }
    return temp_qd;
}
/**
 * @description: MIT坐标系下的力矩转实物坐标系下
 * @param {double} tau MIT坐标系下的力矩矩阵
 * @return {Mat34<double>} 宇树坐标系下的力矩矩阵
 */
Mat34<double> TransformCoordinate::tauMit2unitree(const Mat34<double>& taumit)
{
    Mat34<double> temp_tau = taumit;  // MIT坐标系下的力矩命令
    for (int i = 1; i < 3; i++) {
        for (int j = 0; j < 4; j++) {
            temp_tau(i, j) = -taumit(i, j);
        }
    }
    return temp_tau;
}

/**
 * @description: 宇树坐标系下关节角度转MIT坐标系
 * @param {double} qReal 实物坐标系下的关节角矩阵
 * @return {Mat34<double>} MIT坐标系下的关节角矩阵
 */
Mat34<double> TransformCoordinate::qUnitree2mit(const Mat34<double>& qunitree)
{
    Mat34<double> temp_q = qunitree;  // Unitree坐标系下的绝对位置命令，算法的输出是绝对位置
    for (int i = 1; i < 3; i++) {
        for (int j = 0; j < 4; j++) {
            temp_q(i, j) = -qunitree(i, j);
        }
    }
    return temp_q;
}
/**
 * @description: 宇树坐标系下的关节角速度转MIT坐标系
 * @param {double} qd 宇树坐标系下的关节角速度矩阵
 * @return {Mat34<double>} MIT坐标系下的关节角速度矩阵
 */
Mat34<double> TransformCoordinate::qdUnitree2mit(const Mat34<double>& qdunitree)
{
    Mat34<double> temp_qd = qdunitree;  // Unitree坐标系下的速度命令
    for (int i = 1; i < 3; i++) {
        for (int j = 0; j < 4; j++) {
            temp_qd(i, j) = -qdunitree(i, j);
        }
    }
    return temp_qd;
}
/**
 * @description: 实物坐标系下的力矩转MIT坐标系下
 * @param {double} tau MIT坐标系下的力矩矩阵
 * @return {Mat34<double>} MIT坐标系下的力矩矩阵
 */
Mat34<double> TransformCoordinate::tauUnitree2mit(const Mat34<double>& tauunitree)
{
    Mat34<double> temp_tau = tauunitree;  // Unitree坐标系下的力矩命令
    for (int i = 1; i < 3; i++) {
        for (int j = 0; j < 4; j++) {
            temp_tau(i, j) = -tauunitree(i, j);
        }
    }
    return temp_tau;
}

Mat34<double> TransformCoordinate::qUnitree2real(const Mat34<double>& qunitree, const Mat34<double>& qInitMit, double kneeRatio)
{
    Mat34<double> qMit, qReal, qmit;
    // qMit = qUnitree2mit(qunitree);
    // qReal = qMit2real(qMit, my2mit(qInitMit), kneeRatio);
    qMit = qunitree;
    for (int i = 1; i < 3; i++) {
        for (int j = 0; j < 4; j++) {
            qMit(i, j) = -qunitree(i, j);
        }
    }
    qReal = qMit;
    // qReal -= my2mit(qInitMit);
    qmit = qInitMit;
    for (int i = 0; i < 4; i++) {
        qmit(1, i) = qInitMit(1, i) - M_PI / 2.0;
    }
    qReal -= qmit;

    qReal(1, 1) *= -1;
    qReal(2, 1) *= -1;
    qReal(0, 2) *= -1;
    qReal(0, 3) *= -1;
    qReal(1, 3) *= -1;
    qReal(2, 3) *= -1;

    qReal.row(2) *= kneeRatio;

    return qReal;
}
Mat34<double> TransformCoordinate::qdUnitree2real(const Mat34<double>& qdunitree, double kneeRatio)
{
    Mat34<double> qdMit, qdReal;
    // qdMit = qdUnitree2mit(qdunitree);
    // qdReal = qdMit2real(qdMit, kneeRatio);
    qdMit = qdunitree;
    for (int i = 1; i < 3; i++) {
        for (int j = 0; j < 4; j++) {
            qdMit(i, j) = -qdunitree(i, j);
        }
    }
    qdReal = qdMit;
    qdReal(1, 1) *= -1;
    qdReal(2, 1) *= -1;
    qdReal(0, 2) *= -1;
    qdReal(0, 3) *= -1;
    qdReal(1, 3) *= -1;
    qdReal(2, 3) *= -1;

    qdReal.row(2) *= kneeRatio;  // 考虑减速比

    return qdReal;
}
Mat34<double> TransformCoordinate::tauUnitree2real(const Mat34<double>& tauunitree, double kneeRatio)
{
    Mat34<double> tauMit, tauReal;
    // tauMit = tauUnitree2mit(tauunitree);
    // tauReal = tauMit2real(tauMit, kneeRatio);
    tauMit = tauunitree;
    for (int i = 1; i < 3; i++) {
        for (int j = 0; j < 4; j++) {
            tauMit(i, j) = -tauunitree(i, j);
        }
    }
    tauReal = tauMit;
    tauReal(1, 1) *= -1;
    tauReal(2, 1) *= -1;
    tauReal(0, 2) *= -1;
    tauReal(0, 3) *= -1;
    tauReal(1, 3) *= -1;
    tauReal(2, 3) *= -1;  // webot仿真下的力矩命令

    tauReal.row(2) /= kneeRatio;  // 考虑减速比

    return tauReal;
}

Mat34<double> TransformCoordinate::qReal2unitree(const Mat34<double>& qReal, const Mat34<double>& qInitMit, double kneeRatio)
{
    Mat34<double> qMit, qUnitree, qmit;

    // qMit = qReal2mit(qReal, my2mit(qInitMit), kneeRatio);
    // qUnitree = qMit2unitree(qMit);

    qMit = qReal;
    qMit.row(2) /= kneeRatio;

    qMit(1, 1) *= -1;
    qMit(2, 1) *= -1;
    qMit(0, 2) *= -1;
    qMit(0, 3) *= -1;
    qMit(1, 3) *= -1;
    qMit(2, 3) *= -1;  // MIT坐标系下的相对关节角命令
    // qMit += my2mit(qInitMit);

    qmit = qInitMit;
    for (int i = 0; i < 4; i++) {
        qmit(1, i) = qInitMit(1, i) - M_PI / 2.0;
    }
    qMit += qmit;

    qUnitree = qMit;
    // MIT坐标系下的绝对位置命令

    for (int i = 1; i < 3; i++) {
        for (int j = 0; j < 4; j++) {
            qUnitree(i, j) = -qMit(i, j);
        }
    }

    return qUnitree;
}
Mat34<double> TransformCoordinate::qdReal2unitree(const Mat34<double>& qdReal, double kneeRatio)
{
    Mat34<double> qdMit, qdUnitree;
    // qdMit = qdReal2mit(qdReal, kneeRatio);
    // qdUnitree = qdMit2unitree(qdMit);
    qdMit = qdReal;
    qdMit.row(2) /= kneeRatio;  // 需要的是关节角速度 = 电机角速度 / 同步带减速比
                                // temp_qd = -temp_qd;           // webot(sim)坐标系下的关节角速度

    qdMit(1, 1) *= -1;
    qdMit(2, 1) *= -1;
    qdMit(0, 2) *= -1;
    qdMit(0, 3) *= -1;
    qdMit(1, 3) *= -1;
    qdMit(2, 3) *= -1;  // MIT坐标系下的速度命令
    qdUnitree = qdMit;
    for (int i = 1; i < 3; i++) {
        for (int j = 0; j < 4; j++) {
            qdUnitree(i, j) = -qdMit(i, j);
        }
    }
    return qdUnitree;
}
Mat34<double> TransformCoordinate::tauReal2unitree(const Mat34<double>& tauReal, double kneeRatio)
{
    Mat34<double> tauMit, tauUnitree;
    // tauMit = tauReal2mit(tauReal, kneeRatio);
    // tauUnitree = tauMit2unitree(tauMit);
    tauMit = tauReal;
    tauMit.row(2) *= kneeRatio;  // 需要的是关节的力矩 = 电机力矩 * 同步带减速比

    tauMit(1, 1) *= -1;
    tauMit(2, 1) *= -1;
    tauMit(0, 2) *= -1;
    tauMit(0, 3) *= -1;
    tauMit(1, 3) *= -1;
    tauMit(2, 3) *= -1;  // MIT坐标系下力矩命令
    tauUnitree = tauMit;
    for (int i = 1; i < 3; i++) {
        for (int j = 0; j < 4; j++) {
            tauUnitree(i, j) = -tauMit(i, j);
        }
    }

    return tauUnitree;
}

Mat34<double> TransformCoordinate::qUnitree2Real(const Mat34<double>& qUnitree, const Mat34<double>& qInitUnitree, double kneeRatio)
{
    Mat34<double> qReal = qUnitree;
    qReal -= qInitUnitree;
    qReal(0, 2) *= -1;
    qReal(0, 3) *= -1;
    qReal(1, 0) *= -1;
    qReal(1, 2) *= -1;
    qReal(2, 0) *= -1;
    qReal(2, 2) *= -1;
    qReal.row(2) *= kneeRatio;  // 转换到电机端需要更多关节角，考虑减速比

    return qReal;
}

Mat34<double> TransformCoordinate::qdUnitree2Real(const Mat34<double>& qdUnitree, double kneeRatio)
{
    Mat34<double> qdReal = qdUnitree;
    qdReal(0, 2) *= -1;
    qdReal(0, 3) *= -1;
    qdReal(1, 0) *= -1;
    qdReal(1, 2) *= -1;
    qdReal(2, 0) *= -1;
    qdReal(2, 2) *= -1;
    qdReal.row(2) *= kneeRatio;  // 考虑减速比
    return qdReal;
}

Mat34<double> TransformCoordinate::tauUnitree2Real(const Mat34<double>& tauUnitree, double kneeRatio)
{
    Mat34<double> tauReal = tauUnitree;
    tauReal(0, 2) *= -1;
    tauReal(0, 3) *= -1;
    tauReal(1, 0) *= -1;
    tauReal(1, 2) *= -1;
    tauReal(2, 0) *= -1;
    tauReal(2, 2) *= -1;
    tauReal.row(2) /= kneeRatio;  // 考虑减速比
    return tauReal;
}


Mat34<double> TransformCoordinate::qReal2Unitree(const Mat34<double>& qReal, const Mat34<double>& qInitUnitree, double kneeRatio)
{
    Mat34<double> qUnitree = qReal;
    qUnitree(0, 2) *= -1;
    qUnitree(0, 3) *= -1;
    qUnitree(1, 0) *= -1;
    qUnitree(1, 2) *= -1;
    qUnitree(2, 0) *= -1;
    qUnitree(2, 2) *= -1;
    qUnitree.row(2) /= kneeRatio;  // 转换到电机端需要更多关节角，考虑减速比
    qUnitree += qInitUnitree;

    return qUnitree;
}

Mat34<double> TransformCoordinate::qdReal2Unitree(const Mat34<double>& qdReal, double kneeRatio)
{
    Mat34<double> qdUnitree = qdReal;
    qdUnitree(0, 2) *= -1;
    qdUnitree(0, 3) *= -1;
    qdUnitree(1, 0) *= -1;
    qdUnitree(1, 2) *= -1;
    qdUnitree(2, 0) *= -1;
    qdUnitree(2, 2) *= -1;
    qdUnitree.row(2) /= kneeRatio;  // 考虑减速比

    return qdUnitree;
}

Mat34<double> TransformCoordinate::tauReal2Unitree(const Mat34<double>& tauReal, double kneeRatio)
{
    Mat34<double> tauUnitree = tauReal;
    tauUnitree(0, 2) *= -1;
    tauUnitree(0, 3) *= -1;
    tauUnitree(1, 0) *= -1;
    tauUnitree(1, 2) *= -1;
    tauUnitree(2, 0) *= -1;
    tauUnitree(2, 2) *= -1;
    tauUnitree.row(2) *= kneeRatio;  // 考虑减速比
    return tauUnitree;
}
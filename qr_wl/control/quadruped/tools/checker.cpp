#include "checker.hpp"

#include "mathTools.hpp"

/**
 * @description: 检查命令，若存在错误，则赋值为上一次的命令
 * @param {DataCmd} &motorCmd
 * @param {DataCmd} &motorCmdOld
 * @return {*}
 */
DataCmd &Checker::checkCmd(DataCmd &motorCmd, const Mat43<double> &range)
{
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 4; j++) {
            if (std::isnan(motorCmd.alpha(i, j))) {
                motorCmd.alpha(i, j) = motorCmdOld_.alpha(i, j);
                // std::cout << "motorData.qCmd(" << i << ", " << j << ") = Nan" << std::endl;
            }
            if (std::isnan(motorCmd.torq(i, j))) {
                motorCmd.torq.setZero();
                // std::cout << "motorData.qdCmd(" << i << ", " << j << ") = Nan" << std::endl;
            }
            if (std::isnan(motorCmd.blta(i, j))) {
                motorCmd.blta.setZero();
                // std::cout << "motorData.tauCmd(" << i << ", " << j << ") = Nan" << std::endl;
            }

            if (motorCmd.alpha(i, j) >= range(0, i)) {
                // std::cout << "q(" << i << ", " << j << ") > qLimitUp!" << std::endl;
                // std::cout << "q = " << motorCmd.alpha(i, j) << std::endl;
                motorCmd.alpha(i, j) = range(0, i);

            } else if (motorCmd.alpha(i, j) <= range(1, i)) {
                // std::cout << "q(" << i << ", " << j << ") < qLimitDown!" << std::endl;
                // std::cout << "q = " << motorCmd.alpha(i, j) << std::endl;
                motorCmd.alpha(i, j) = range(1, i);
            }

            if (motorCmd.blta(i, j) >= range(2, i)) {
                // cout << "tau(" << i << ", " << j << ") > tauLimitUp!" << endl;
                // cout << "tau = " << (*tau)(i, j)  << endl;
                motorCmd.blta(i, j) = range(2, i);

            } else if (motorCmd.blta(i, j) <= range(3, i)) {
                // cout << "tau(" << i << ", " << j << ") < tauLimitDown!" << endl;
                // cout << "tau = " << (*tau)(i, j)  << endl;
                motorCmd.blta(i, j) = range(3, i);
            }
        }
    }

    motorCmdOld_ = motorCmd;

    return motorCmd;
}

DataCmd &Checker::Lie(DataCmd &motorCmd)
{
    Mat43<double> cmdRange;    // qMax; qMin; tauMax tauMin
    cmdRange << 80, 150, 155,  // qMax
        -80, -80, 2,           // qMin
        36, 36, 36,            // tauMax
        -36, -36, -36;         // tauMin

    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 3; j++) {
            cmdRange(i, j) = MathFun::deg2rad(cmdRange(i, j));
        }
    }
    return checkCmd(motorCmd, cmdRange);
}

DataCmd &Checker::StandUp(DataCmd &motorCmd)
{
    Mat43<double> cmdRange;    // qMax; qMin; tauMax tauMin
    cmdRange << 80, 150, 155,  // qMax
        -80, -80, 2,           // qMin
        36, 36, 36,            // tauMax
        -36, -36, -36;         // tauMin

    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 3; j++) {
            cmdRange(i, j) = MathFun::deg2rad(cmdRange(i, j));
        }
    }
    return checkCmd(motorCmd, cmdRange);
};

DataCmd &Checker::Stand(DataCmd &motorCmd)
{
    Mat43<double> cmdRange;    // qMax; qMin; tauMax tauMin
    cmdRange << 80, 150, 155,  // qMax
        -80, -80, 2,           // qMin
        36, 36, 36,            // tauMax
        -36, -36, -36;         // tauMin

    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 3; j++) {
            cmdRange(i, j) = MathFun::deg2rad(cmdRange(i, j));
        }
    }
    return checkCmd(motorCmd, cmdRange);
};
DataCmd &Checker::LieDown(DataCmd &motorCmd)
{
    Mat43<double> cmdRange;    // qMax; qMin; tauMax tauMin
    cmdRange << 80, 150, 155,  // qMax
        -80, -80, 2,           // qMin
        36, 36, 36,            // tauMax
        -36, -36, -36;         // tauMin

    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 3; j++) {
            cmdRange(i, j) = MathFun::deg2rad(cmdRange(i, j));
        }
    }
    return checkCmd(motorCmd, cmdRange);
};
DataCmd &Checker::Walk(DataCmd &motorCmd)
{
    Mat43<double> cmdRange;    // qMax; qMin; tauMax tauMin
    cmdRange << 80, 150, 155,  // qMax
        -80, -80, 2,           // qMin
        36, 36, 36,            // tauMax
        -36, -36, -36;         // tauMin

    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 3; j++) {
            cmdRange(i, j) = MathFun::deg2rad(cmdRange(i, j));
        }
    }

    // cout << cmdRange << endl;

    return checkCmd(motorCmd, cmdRange);
};
DataCmd &Checker::Fall(DataCmd &motorCmd)
{
    Mat43<double> cmdRange;    // qMax; qMin; tauMax tauMin
    cmdRange << 80, 150, 155,  // qMax
        -80, -80, 2,           // qMin
        36, 36, 36,            // tauMax
        -36, -36, -36;         // tauMin

    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 3; j++) {
            cmdRange(i, j) = MathFun::deg2rad(cmdRange(i, j));
        }
    }
    return checkCmd(motorCmd, cmdRange);
};
DataCmd &Checker::Recover(DataCmd &motorCmd)
{
    Mat43<double> cmdRange;    // qMax; qMin; tauMax tauMin
    cmdRange << 80, 150, 155,  // qMax
        -80, -80, 2,           // qMin
        36, 36, 36,            // tauMax
        -36, -36, -36;         // tauMin

    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 3; j++) {
            cmdRange(i, j) = MathFun::deg2rad(cmdRange(i, j));
        }
    }
    return checkCmd(motorCmd, cmdRange);
};

DataCmd &Checker::BackFlip(DataCmd &motorCmd)
{
    Mat43<double> cmdRange;    // qMax; qMin; tauMax tauMin
    cmdRange << 80, 150, 155,  // qMax
        -80, -80, 2,           // qMin
        36, 36, 36,            // tauMax
        -36, -36, -36;         // tauMin

    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 3; j++) {
            cmdRange(i, j) = MathFun::deg2rad(cmdRange(i, j));
        }
    }
    return checkCmd(motorCmd, cmdRange);
};

/**
 * @description: 数据转化
 *
 *      leg0  leg1  leg2  leg3
 * abad   0    1     2     3
 * hip    4    5     6     7
 * knee   8    9     10     11
 *
 * @return {}
 */
std::optional<Mat34<double>> VecToMat34(const std::vector<double> &set)
{
    if (set.size() != 12) {
        return std::nullopt;
    }

    Mat34<double> ret;
    ret(0, 0) = set[0];
    ret(0, 1) = set[1];
    ret(0, 2) = set[2];
    ret(0, 3) = set[3];

    ret(1, 0) = set[4];
    ret(1, 1) = set[5];
    ret(1, 2) = set[6];
    ret(1, 3) = set[7];

    ret(2, 0) = set[8];
    ret(2, 1) = set[9];
    ret(2, 2) = set[10];
    ret(2, 3) = set[11];

    return ret;
}

std::vector<double> Mat34ToVec(const Mat34<double> &set)
{
    std::vector<double> ret;
    ret.push_back(set(0, 0));
    ret.push_back(set(0, 1));
    ret.push_back(set(0, 2));
    ret.push_back(set(0, 3));

    ret.push_back(set(1, 0));
    ret.push_back(set(1, 1));
    ret.push_back(set(1, 2));
    ret.push_back(set(1, 3));

    ret.push_back(set(2, 0));
    ret.push_back(set(2, 1));
    ret.push_back(set(2, 2));
    ret.push_back(set(2, 3));
    return ret;
}
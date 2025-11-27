#pragma once

#include "baseline.hpp"

class TransformCoordinate
{
public:
    // 注意这里实际使用时，使用嵌套调用，my2mit(qInit)
    Mat34<double> qMit2real(const Mat34<double>& q, const Mat34<double>& qInitMit, double kneeRatio);
    Mat34<double> qdMit2real(const Mat34<double>& qd, double kneeRatio);
    Mat34<double> tauMit2real(const Mat34<double>& tau, double kneeRatio);

    Mat34<double> qReal2mit(const Mat34<double>& qReal, const Mat34<double>& qInitMit, double kneeRatio);
    Mat34<double> qdReal2mit(const Mat34<double>& qdReal, double kneeRatio);
    Mat34<double> tauReal2mit(const Mat34<double>& tauReal, double kneeRatio);

    Mat34<double> my2mit(const Mat34<double>& qmy);
    Mat34<double> mit2my(const Mat34<double>& qmit);

    Mat34<double> qMit2unitree(const Mat34<double>& qunitree);
    Mat34<double> qdMit2unitree(const Mat34<double>& qdunitree);
    Mat34<double> tauMit2unitree(const Mat34<double>& tauunitree);

    Mat34<double> qUnitree2mit(const Mat34<double>& qmit);
    Mat34<double> qdUnitree2mit(const Mat34<double>& qdmit);
    Mat34<double> tauUnitree2mit(const Mat34<double>& taumit);

    Mat34<double> qUnitree2real(const Mat34<double>& qunitree, const Mat34<double>& qInitMit, double kneeRatio);
    Mat34<double> qdUnitree2real(const Mat34<double>& qdunitree, double kneeRatio);
    Mat34<double> tauUnitree2real(const Mat34<double>& tauunitree, double kneeRatio);

    Mat34<double> qReal2unitree(const Mat34<double>& qReal, const Mat34<double>& qInitMit, double kneeRatio);
    Mat34<double> qdReal2unitree(const Mat34<double>& qdReal, double kneeRatio);
    Mat34<double> tauReal2unitree(const Mat34<double>& tauReal, double kneeRatio);

    Mat34<double> qUnitree2Real(const Mat34<double>& qUnitree, const Mat34<double>& qInitUnitree, double kneeRatio);
    Mat34<double> qdUnitree2Real(const Mat34<double>& qdUnitree, double kneeRatio);
    Mat34<double> tauUnitree2Real(const Mat34<double>& tauUnitree, double kneeRatio);

    Mat34<double> qReal2Unitree(const Mat34<double>& qReal, const Mat34<double>& qInitUnitree, double kneeRatio);
    Mat34<double> qdReal2Unitree(const Mat34<double>& qdReal, double kneeRatio);
    Mat34<double> tauReal2Unitree(const Mat34<double>& tauReal, double kneeRatio);

};
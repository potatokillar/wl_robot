

#pragma once

#include <map>

#include "ArmType.hpp"
#include "baseline.hpp"
#include "innerType.hpp"

// 基类配置选项
struct CfgArm_base
{
    ArmModel model = ArmModel::base;
    NetMode mode = NetMode::base;
    double ArmAixsNum = 6;
    struct TOOL
    {
        Pose tool{};
    } tool;
    struct BASE
    {
        Pose base{};
    } base;
    struct KINE_PARAM
    {
        std::vector<double> theta;
        std::vector<double> d;
        std::vector<double> a;
        std::vector<double> alpha;
        std::vector<double> offset;
        // Vec7<double> theta_7;
        // Vec7<double> d_7;
        // Vec7<double> a_7;
        // Vec7<double> alpha_7;
        // Vec7<double> offset_7;
    } kine_param;

    struct INERTIA
    {
        double Icxx, Icyy, Iczz, Icxy, Icxz, Icyz;
    } inertia;
    struct DYNA_PARAM
    {
        std::vector<double> M;
        std::vector<INERTIA> Ic;
        std::vector<Vec3<double>> Mc;
        std::vector<double> Ia;
        std::vector<double> fv;
        std::vector<double> fc;
        // Vec7<double> M_7;

    } dyna_param;

    struct AXIS
    {
        std::vector<double> S_min;
        std::vector<double> S_max;
        std::vector<double> S_min_user;
        std::vector<double> S_max_user;
        std::vector<double> S_min_real;
        std::vector<double> S_max_real;
        std::vector<double> S_min_reset;
        std::vector<double> S_max_reset;
        std::vector<double> V_max;
        std::vector<double> A_max;
        std::vector<double> J_max;
        std::vector<double> gear_ratio;
        std::vector<double> gear_flip;

        // Vec7<double> S_min_7;
        // Vec7<double> S_max_7;
        // Vec7<double> S_min_user_7;
        // Vec7<double> S_max_user_7;
        // Vec7<double> S_min_real_7;
        // Vec7<double> S_max_real_7;
        // Vec7<double> S_min_reset_7;
        // Vec7<double> S_max_reset_7;
        // Vec7<double> V_max_7;
        // Vec7<double> A_max_7;
        // Vec7<double> J_max_7;
        // Vec7<double> gear_ratio_7;
        // Vec7<bool> gear_flip_7;

    } axis;
    struct Coordinate
    {
        std::map<std::string, Pose> tool;

    } coor;
    struct DESC
    {
    } desc;
    struct Cartesian
    {
        Vec6<double> V_cart;
        Vec6<double> A_cart;
        Vec6<double> J_cart;
    } cart;

    // enum class ArmMode
    // {
    //     base,
    //     Wl,
    //     Iris,
    // };

    CfgArm_base()
    {
        tool.tool.setZero();
        base.base.setZero();
        // kine_param.theta.setZero();
        // kine_param.d.setZero();
        // kine_param.a.setZero();
        // kine_param.alpha.setZero();
        // kine_param.offset.setZero();
        // kine_param.theta_7.setZero();
        // kine_param.d_7.setZero();
        // kine_param.a_7.setZero();
        // kine_param.alpha_7.setZero();
        // kine_param.offset_7.setZero();

        // dyna_param.M.setZero();
        // dyna_param.M_7.setZero();

        // axis.S_min.setZero();
        // axis.S_max.setZero();
        // axis.V_max.setZero();
        // axis.A_max.setZero();
        // axis.J_max.setZero();
        // axis.gear_ratio.setZero();
        // axis.gear_flip.setZero();
        // axis.S_min_7.setZero();
        // axis.S_max_7.setZero();
        // axis.V_max_7.setZero();
        // axis.A_max_7.setZero();
        // axis.J_max_7.setZero();
        // axis.gear_flip_7.setZero();

        cart.V_cart.setZero();
        cart.A_cart.setZero();
        cart.J_cart.setZero();

        // ArmMode armmode = ArmMode::base;
    }

    template <typename T>
    void Load(const T& in)
    {
        *this = static_cast<CfgArm_base>(in);
    }
};

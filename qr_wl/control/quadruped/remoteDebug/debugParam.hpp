
#pragma once
#include <vector>

#include "baseline.hpp"
#include "debug.hpp"

class DebugParam final
{
public:
    static DebugParam& GetInstance()
    {
        static DebugParam instance;
        return instance;
    }

public:
    double A{0};
    double B{0};
    double C{0};
    double D{0};
    double E{0};
    double F{0};
    double G{0};
    double H{0};
    double I{0};
    double J{0};

private:
    DebugParam();
    ValueRange<double> range_{-10000, 10000};

    bool Set(int idx, const std::vector<double>& set);
    std::vector<double> Get(int idx);
    ParamInfo GetInfo(int idx);

    bool set_A(std::vector<double> set);
    std::vector<double> get_A();
    ParamInfo get_A_info();

    bool set_B(std::vector<double> set);
    std::vector<double> get_B();
    ParamInfo get_B_info();

    bool set_C(std::vector<double> set);
    std::vector<double> get_C();
    ParamInfo get_C_info();

    bool set_D(std::vector<double> set);
    std::vector<double> get_D();
    ParamInfo get_D_info();

    bool set_E(std::vector<double> set);
    std::vector<double> get_E();
    ParamInfo get_E_info();

    bool set_F(std::vector<double> set);
    std::vector<double> get_F();
    ParamInfo get_F_info();

    bool set_G(std::vector<double> set);
    std::vector<double> get_G();
    ParamInfo get_G_info();

    bool set_H(std::vector<double> set);
    std::vector<double> get_H();
    ParamInfo get_H_info();

    bool set_I(std::vector<double> set);
    std::vector<double> get_I();
    ParamInfo get_I_info();

    bool set_J(std::vector<double> set);
    std::vector<double> get_J();
    ParamInfo get_J_info();
};

inline DebugParam& GetRemoteParam() { return DebugParam::GetInstance(); }
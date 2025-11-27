
#include "debugParam.hpp"

DebugParam::DebugParam()
{
    contrParamHotUpdate
        .AddParamNum("param-A", [this](std::vector<double> set) { return this->set_A(set); }, [this]() { return this->get_A(); }, [this]() { return this->get_A_info(); });
    contrParamHotUpdate
        .AddParamNum("param-B", [this](std::vector<double> set) { return this->set_B(set); }, [this]() { return this->get_B(); }, [this]() { return this->get_B_info(); });
    contrParamHotUpdate
        .AddParamNum("param-C", [this](std::vector<double> set) { return this->set_C(set); }, [this]() { return this->get_C(); }, [this]() { return this->get_C_info(); });

    contrParamHotUpdate
        .AddParamNum("param-D", [this](std::vector<double> set) { return this->set_D(set); }, [this]() { return this->get_D(); }, [this]() { return this->get_D_info(); });
    contrParamHotUpdate
        .AddParamNum("param-E", [this](std::vector<double> set) { return this->set_E(set); }, [this]() { return this->get_E(); }, [this]() { return this->get_E_info(); });
    contrParamHotUpdate
        .AddParamNum("param-F", [this](std::vector<double> set) { return this->set_F(set); }, [this]() { return this->get_F(); }, [this]() { return this->get_F_info(); });
    contrParamHotUpdate
        .AddParamNum("param-G", [this](std::vector<double> set) { return this->set_G(set); }, [this]() { return this->get_G(); }, [this]() { return this->get_G_info(); });
    contrParamHotUpdate
        .AddParamNum("param-H", [this](std::vector<double> set) { return this->set_H(set); }, [this]() { return this->get_H(); }, [this]() { return this->get_H_info(); });
    contrParamHotUpdate
        .AddParamNum("param-I", [this](std::vector<double> set) { return this->set_I(set); }, [this]() { return this->get_I(); }, [this]() { return this->get_I_info(); });
    contrParamHotUpdate
        .AddParamNum("param-J", [this](std::vector<double> set) { return this->set_J(set); }, [this]() { return this->get_J(); }, [this]() { return this->get_J_info(); });
}

bool DebugParam::Set(int idx, const std::vector<double>& set)
{
    if (set.size() < 1) {
        return false;
    }

    if ((set[0] < range_.min) || (set[0] > range_.max)) {
        return false;
    }
    switch (idx) {
        case 0:
            A = set[0];
            break;
        case 1:
            B = set[0];
            break;
        case 2:
            C = set[0];
            break;
        case 3:
            D = set[0];
            break;
        case 4:
            E = set[0];
            break;
        case 5:
            F = set[0];
            break;
        case 6:
            G = set[0];
            break;
        case 7:
            H = set[0];
            break;
        case 8:
            I = set[0];
            break;
        case 9:
            J = set[0];
            break;
        default:
            break;
    }
    return true;
}
std::vector<double> DebugParam::Get(int idx)
{
    std::vector<double> ret;
    ret.resize(1);

    switch (idx) {
        case 0:
            ret[0] = A;
            break;
        case 1:
            ret[0] = B;
            break;
        case 2:
            ret[0] = C;
            break;
        case 3:
            ret[0] = D;
            break;
        case 4:
            ret[0] = E;
            break;
        case 5:
            ret[0] = F;
            break;
        case 6:
            ret[0] = G;
            break;
        case 7:
            ret[0] = H;
            break;
        case 8:
            ret[0] = I;
            break;
        case 9:
            ret[0] = J;
            break;
        default:
            ret[0] = 0;
            break;
    }
    return ret;
}
ParamInfo DebugParam::GetInfo(int idx)
{
    (void)idx;
    ParamInfo info;
    info.type = ParamType::range;
    info.num.push_back(range_.min);
    info.num.push_back(range_.max);
    return info;
}

///////////////////////////////////
//-A
bool DebugParam::set_A(std::vector<double> set) { return Set(0, set); }
std::vector<double> DebugParam::get_A() { return Get(0); }
ParamInfo DebugParam::get_A_info() { return GetInfo(0); }

//-B
bool DebugParam::set_B(std::vector<double> set) { return Set(1, set); }
std::vector<double> DebugParam::get_B() { return Get(1); }
ParamInfo DebugParam::get_B_info() { return GetInfo(1); }

//-C
bool DebugParam::set_C(std::vector<double> set) { return Set(2, set); }
std::vector<double> DebugParam::get_C() { return Get(2); }
ParamInfo DebugParam::get_C_info() { return GetInfo(2); }

//-D
bool DebugParam::set_D(std::vector<double> set) { return Set(3, set); }
std::vector<double> DebugParam::get_D() { return Get(3); }
ParamInfo DebugParam::get_D_info() { return GetInfo(3); }

//-E
bool DebugParam::set_E(std::vector<double> set) { return Set(4, set); }
std::vector<double> DebugParam::get_E() { return Get(4); }
ParamInfo DebugParam::get_E_info() { return GetInfo(4); }

//-F
bool DebugParam::set_F(std::vector<double> set) { return Set(5, set); }
std::vector<double> DebugParam::get_F() { return Get(5); }
ParamInfo DebugParam::get_F_info() { return GetInfo(5); }

//-G
bool DebugParam::set_G(std::vector<double> set) { return Set(6, set); }
std::vector<double> DebugParam::get_G() { return Get(6); }
ParamInfo DebugParam::get_G_info() { return GetInfo(6); }

// H
bool DebugParam::set_H(std::vector<double> set) { return Set(7, set); }
std::vector<double> DebugParam::get_H() { return Get(7); }
ParamInfo DebugParam::get_H_info() { return GetInfo(7); }

//-I
bool DebugParam::set_I(std::vector<double> set) { return Set(8, set); }
std::vector<double> DebugParam::get_I() { return Get(8); }
ParamInfo DebugParam::get_I_info() { return GetInfo(8); }

//-J
bool DebugParam::set_J(std::vector<double> set) { return Set(9, set); }
std::vector<double> DebugParam::get_J() { return Get(9); }
ParamInfo DebugParam::get_J_info() { return GetInfo(9); }
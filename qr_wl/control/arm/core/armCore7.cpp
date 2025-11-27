
#include "armCore7.hpp"

#include "armParam.hpp"
#include "innerType.hpp"

using namespace std;
using namespace arm;

ArmCore7::ArmCore7(const ArmBootArgs& armArgs) : ArmCore(armArgs)
{
    coData_ = make_shared<CoreData>();
    coData_->armSta = ArmState::init;
    // ArmParam();

    //  均来自用户侧的指令，非阻塞
    rpcRxDeal_.emplace_back("arm::move", [this](MsgType set) { return this->RxMoveSearch(set); });
    rpcRxDeal_.emplace_back("arm::txrx_callback", [this](MsgType set) { return this->RxTxRxCallback(set); });
    rpcRxDeal_.emplace_back("arm::info_callback", [this](MsgType set) { return this->RxInfoCallback(set); });
    rpcRxDeal_.emplace_back("arm::set_position_limit", [this](MsgType set) { return this->RxSetPositionLimit(set); });
    rpcRxDeal_.emplace_back("arm::set_joint_acc_limit", [this](MsgType set) { return this->RxSetJointAccLimit(set); });
    rpcRxDeal_.emplace_back("arm::set_joint_speed_limit", [this](MsgType set) { return this->RxSetJointSpeedLimit(set); });
    for (auto& rpc : rpcRxDeal_) {
        rpc.Connect();
    }
    coData_->armSta = ArmState::noReady;
}

/**
 * @description: 本函数周期并不是严格的4ms，因为里面存在RPC的发送，而RPC发送一定是阻塞的
 * @return {}
 */
void ArmCore7::Run()
{
    ArmCore::Run();
    {
        PubInfo();
    }
}

/**
 * @description: core上报机械臂用户设置的参数
 */

/**
 * @description: core上报信息，机械臂的实时信息都通过这个获取，不再存在单独的获取指令
 * @todo Vec6改为六轴七轴通用的vector
 * @return {}
 */
void ArmCore7::PubInfo()
{
    ArmUserInfo info;
    info.angle.clear();
    for (int i = 0; i < 7; i++) {
        info.angle.push_back(coData_->nowAngle_seven(i));  //.push_back();
        info.velocity.push_back(coData_->nowVelocity_seven(i));
    }
    Pose ToolValue = user_.userTool[coData_->toolName];

    if (GetArmParam().ArmAixsNum == 7) {
        auto FK_seven = math_.Fkine_seven(coData_->nowAngle_seven, ToolValue);
        info.cart = FK_seven.first;
        info.Phi = FK_seven.second;
    }

    info.state = coData_->armSta;
    info.ioInSta = coData_->motorInfo.ioInSta;
    info.ioOutSta = coData_->motorInfo.ioOutSta;
    // info.motor.current = coData_->motorInfo.errCode; // todo电压电流数据未实现

    MsgTrySend("arm::pub_info", info);
}

/**
 * @description: 发送接收回调，由motor线程调用
 * @param ret
 * @todo Vec6改为六轴七轴通用的vector
 * @return {}
 */
std::optional<msg::arm_cmd> ArmCore7::TxRxCallback(const msg::arm_data& ret)
{
    //////////接收处理///////////////////
    for (int i = 0; i < 7; i++) {
        coData_->nowAngle_seven(i) = ret.motor[i].alpha * GetArmParam().axis.gear_flip[i];  // real
        coData_->nowVelocity_seven(i) = ret.motor[i].torq * GetArmParam().axis.gear_flip[i];
    }

    /////////发送处理////////////////////////
    // running状态才能发送
    if (GetArmState() != ArmState::running) {
        return std::nullopt;
    }

    // 空容器不发送
    if (moveData_.datas.empty()) {
        return std::nullopt;
    }

    msg::arm_cmd cmd;
    if (moveData_.idx < GetFirstMoveData().size()) {
        vector<double> qcmd;
        qcmd.clear();
        for (int i = 0; i < 7; i++) {
            qcmd.push_back(GetFirstMoveData()[moveData_.idx].motor[i].alpha);
        }

        // cout << " cmd:" << qcmd << endl;
        bool ret = true;
        ret = math_.CheckAngle(qcmd, GetArmParam().axis.S_min_real, GetArmParam().axis.S_max_real);
        if (ret == false) {
            ClearFirstMoveData();
            LOG_WARN_ARM("CheckAngle fail, q > qLimit");
            SetArmState(ArmState::error);
            return std::nullopt;
        } else {
            for (int slc = 1; slc <= 7; slc++) {
                cmd.motor[slc - 1].alpha = qcmd[slc - 1];
            }
            moveData_.idx++;
        }
        return cmd;
    }
    return std::nullopt;
}

/**
 * @description: motor其他信息上报回调
 * @param info
 * @return {}
 */
void ArmCore7::InfoCallback(const msg::arm_motor_info& info)
{
    coData_->motorInfo = info;
    // LOG_DEBUG_ARM("rx motor state: {}", Enum2Num(info.state));
}

/**
 * @description: 接收阻塞任务
 * @param in
 * @todo 关节规划根据数据长度判断，笛卡尔规划改为根据型号判断
 * @return {}
 */
MsgType ArmCore7::RxMoveSearch(const MsgType& in)
{
    Result<u32> Tms;
    auto set = in.GetType<ArmMovePoint>();
    Result<double> T;
    T.second = 0;
    planlist_.clear();
    switch (set.flag) {
        case MoveFlag::movej: {
            std::vector<double> Targetjoint = any_cast<std::vector<double>>(set.Targetjoint);
            Vec7<double> Target;
            for (int i = 0; i < 7; i++) {
                Target(i) = Targetjoint[i];
            }
            LOG_DEBUG_ARM("JointSet: {:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}", Target(0), Target(1), Target(2), Target(3), Target(4), Target(5), Target(6));
            T = MoveJ_t(Target, set.speed, set.acceleration, set.movemode, set.flag);
            // cout << "MoveJ_t:" << ret << endl;
            break;
        }
        case MoveFlag::movel: {
            LOG_DEBUG_ARM("PoseSet: {:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}",
                          set.Targetpose.tran_.x,
                          set.Targetpose.tran_.y,
                          set.Targetpose.tran_.z,
                          set.Targetpose.rpy_.rx,
                          set.Targetpose.rpy_.ry,
                          set.Targetpose.rpy_.rz);
            T = MoveL_t(set.Targetpose, set.speed, set.acceleration, coData_->toolName, set.movemode, set.flag);
            // cout << "MoveL_t:" << Enum2Num(T.first) << endl;

            break;
        }
        default:
            break;
    }

    if (T.first == RetState::ok) {
        Tms.second = T.second * 1.1 * 1000;  // 增加1.1系数表示最长时间
        LOG_INFO_ARM("Task start, size:{}, time:{}", moveData_.datas.size(), Tms.second);
        Tms.first = RetState::ok;
    } else {
        Tms.first = T.first;
    }
    // cout << "Tms:" << Enum2Num(Tms.first) << endl;
    return Tms;
}

void ArmCore7::SaveAngle_seven(Vec7<double> q)
{
    msg::arm_cmd cmd;

    for (int i = 0; i < 7; i++) {
        cmd.motor[i].alpha = q(i) * GetArmParam().axis.gear_flip[i];
    }
    planlist_.push_back(cmd);
    // for (int i = 0; i < 6; i++) {
    //     cmd.motor[i].alpha = q(i);
    // }
    // cmd.motor[1].alpha = -q(1);
    // cmd.motor[4].alpha = -q(4);
    // moveData_.data.push_back(cmd);
}

double ArmCore7::ComputeT(Vec6<double> S, Vec6<double> V, Vec6<double> Amax, MoveFlag flag)
{
    Vec6<double> tmp1, tmp2, vlim, Ta, Td, Tv, T;
    double Tzy = 0, tmpT = 0, zy = 7;

    for (int i = 0; i < 6; i++) {
        S(i) = fabs(S(i));
        tmp1(i) = std::pow(V(i), 2);
        tmp2(i) = S(i) * Amax(i);
        if (tmp1(i) <= tmp2(i)) {
            vlim(i) = V(i);
            Ta(i) = vlim(i) / Amax(i);
            Tv(i) = S(i) - (std::pow(vlim(i), 2) / Amax(i));
            T(i) = 2 * Ta(i) + Tv(i);
        } else {
            vlim(i) = sqrt(S(i) * Amax(i));
            Ta(i) = vlim(i) / Amax(i);
            T(i) = 2 * Ta(i);
        }
    }
    // cout << "S:" << S << endl;
    // cout << "T_:" << T << endl;
    tmpT = T(0);
    if (flag == MoveFlag::movej) {
        for (int i = 0; i < 6; i++) {
            if (T(i) >= tmpT) {
                tmpT = T(i);
                zy = i;
            }
        }
    } else if (flag == MoveFlag::movel) {
        for (int i = 0; i < 4; i++) {
            if (T(i) >= tmpT) {
                tmpT = T(i);
                zy = i;
            }
        }
    }

    if (zy == 7) {
        Tzy = 0;
    } else {
        Tzy = T(zy);
    }
    if (Tzy < 0.0001) {
        Tzy = 0.04;
    }
    return Tzy;
}

double ArmCore7::ComputeT_seven(Vec7<double> S, Vec7<double> V, Vec7<double> Amax, MoveFlag flag)
{
    Vec7<double> tmp1, tmp2, vlim, Ta, Td, Tv, T;
    double Tzy = 0, tmpT = 0, zy = 8;

    for (int i = 0; i < 7; i++) {
        S(i) = fabs(S(i));
        tmp1(i) = std::pow(V(i), 2);
        tmp2(i) = S(i) * Amax(i);
        if (tmp1(i) <= tmp2(i)) {
            vlim(i) = V(i);
            Ta(i) = vlim(i) / Amax(i);
            Tv(i) = S(i) - (std::pow(vlim(i), 2) / Amax(i));
            T(i) = 2 * Ta(i) + Tv(i);
        } else {
            vlim(i) = sqrt(S(i) * Amax(i));
            Ta(i) = vlim(i) / Amax(i);
            T(i) = 2 * Ta(i);
        }
    }
    // cout << "S:" << S << endl;
    // cout << "T_:" << T << endl;
    tmpT = T(0);
    if (flag == MoveFlag::movej) {
        for (int i = 0; i < 7; i++) {
            if (T(i) >= tmpT) {
                tmpT = T(i);
                zy = i;
            }
        }
    }

    if (zy == 8) {
        Tzy = 0;
    } else {
        Tzy = T(zy);
    }
    if (Tzy < 0.0001) {
        Tzy = 0.04;
    }
    return Tzy;
}

Result<double> ArmCore7::MoveJ_t(Vec7<double> Targetjoint, double userV, double userA, MoveMode moveMode, MoveFlag flag)  // ABS
{
    // cout << "7" << endl;
    Vec7<double> jointStart, jointEnd, S, q_cmd, qd_cmd, S_plan;
    Vec7<double> Amax, Vmax, Vuser, Auser;
    double Plan_T = 0.0;
    Result<double> T;
    Result<Vec7<double>> speed, acc;
    bool Needmove = false;
    T.first = RetState::error;
    T.second = 0;
    if (moveData_.datas.empty()) {
        jointStart = coData_->nowAngle_seven;
    } else {
        // cout << "datas2:" << moveData_.datas.size() << endl;
        for (int i = 0; i < 7; i++) {
            jointStart(i) = moveData_.datas.back().back().motor[i].alpha * GetArmParam().axis.gear_flip[i];
        }
    }
    cout << "jointStart:" << jointStart << endl;
    // auto& armParam = GetArmParam();
    for (int i = 0; i < 7; i++) {
        Vmax(i) = user_.Vmax[i];
        Amax(i) = user_.Amax[i];
    }
    // Vmax = armParam.axis.V_max;
    // Amax = armParam.axis.A_max;
    LOG_DEBUG_ARM("jointStart: {:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}",
                  jointStart(0),
                  jointStart(1),
                  jointStart(2),
                  jointStart(3),
                  jointStart(4),
                  jointStart(5),
                  jointStart(6));

    userV = userV * user_.rapidrate;  // 用户设置的速度绝对值按全局要求的比例进行控制
    userA = userA * user_.rapidrate;
    speed = math_.CheckSpeed_seven(userV, Vmax);  // 用户指定速度检查是否超限，映射给六个关节
    acc = math_.CheckAcc_seven(userA, Amax, flag);
    if (speed.first == RetState::ok && acc.first == RetState::ok) {
        Vuser = speed.second;
        Auser = acc.second;
    } else {
        T.first = speed.first;
        return T;
    }
    if (moveMode == MoveMode::abs) {
        jointEnd = Targetjoint;
    } else if (moveMode == MoveMode::incr) {
        jointEnd = jointStart + Targetjoint;
    }
    vector<double> check_end;
    for (int i = 0; i < 7; i++) {
        check_end.push_back(jointEnd(i));
    }
    bool ret = math_.CheckAngle_seven(check_end, user_.Smin, user_.Smax);  // 检查用户输入角度是否超出限位
    if (ret == false) {
        T.first = RetState::outRange;
    } else {
        S = jointEnd - jointStart;
        for (int i = 0; i < 7; i++) {
            if (fabs(S(i)) > 1e-4) {
                Needmove = true;
            }
        }
        if (Needmove == false) {
            T.first = RetState::ok;
            moveData_.datas.clear();
        } else {
            T.second = ComputeT_seven(S, Vuser, Auser, flag);
            Plan_T = T.second / dt_;
            cout << "T " << Plan_T << " " << dt_ << endl;
            // LOG_DEBUG("[ArmTask] T: {:.4f},{}", T.second, dt_);
            for (double tpl = 0; tpl <= Plan_T; tpl++) {
                for (int i = 0; i < 7; i++) {
                    S(i) = jointEnd(i) - jointStart(i);
                    S(i) = math_.Plan5(S(i), Plan_T * dt_, tpl * dt_).Y;
                    q_cmd(i) = jointStart(i) + S(i);
                    qd_cmd(i) = (q_cmd(i) - qOldseven_(i)) / dt_;
                    qOldseven_(i) = q_cmd(i);
                };
                SaveAngle_seven(q_cmd);
            }
            if (moveData_.datas.empty()) {
                moveData_.datas.emplace_back(vector<msg::arm_cmd>{});
            }
            moveData_.datas.push_back(planlist_);
            T.first = RetState::ok;
        }
    }
    return T;
}

Result<double> ArmCore7::MoveL_t(Pose Targetpose, double userV, double userA, std::string ToolName, MoveMode moveMode, MoveFlag flag)
{
    // cout << "7" << endl;
    Pose POSERecv, POSEND, POSE_start;
    CartesianTran POS_now, POS_end, POSEend;
    Rpy ENDRPY, RPY_now, RPY_end;

    std::pair<Pose, double> KPS44, KPSrecv;
    double Ps, Qs, Plan_T;
    Quaternion Quat_now, Quat_end, Quat;
    Vec6<double> S, Vmax, Amax, Vuser, Auser;
    Vec7<double> q_cmd, qd_cmd;
    vector<armPoint> sendData;
    Result<double> T;
    bool Needmove = false;
    Result<Vec6<double>> speed, acc;

    for (int i = 0; i < 6; i++) {
        Vmax[i] = user_.Vmax[i];
        Amax[i] = user_.Amax[i];
    }

    userV = userV * rapidrate_;  // 用户设置的速度绝对值按全局要求的比例进行控制
    userA = userA * user_.rapidrate;

    speed = math_.CheckSpeed(userV, Vmax);  // 用户指定速度检查是否超限，映射给线速度和姿态，
                                            // 如果同时动位置和姿态，可能速度单位有问题，此时仍需要进行优化
    acc = math_.CheckAcc(userA, Amax, flag);
    // cout << "Speed:" << Enum2Num(speed.first) << "acc:" << Enum2Num(acc.first) << endl;
    if (speed.first == RetState::ok && acc.first == RetState::ok) {
        Vuser = speed.second;
        Auser = acc.second;
    } else {
        T.first = speed.first;
        return T;
    }
    Pose toolValue = user_.userTool.find(ToolName)->second;
    if (moveData_.datas.empty()) {
        // cout << "datas1:" << moveData_.datas.size() << endl;
        for (int i = 0; i < 7; i++) {
            qRecvseven_(i) = coData_->nowAngle_seven[i];
        }

    } else {
        // cout << "datas2:" << moveData_.datas.size() << endl;
        for (int i = 0; i < 7; i++) {
            qRecvseven_(i) = moveData_.datas.back().back().motor[i].alpha * GetArmParam().axis.gear_flip[i];
        }
    }

    // qRecvseven_ = coData_->nowAngle_seven;
    KPSrecv = math_.Fkine_seven(qRecvseven_, toolValue);
    POSERecv = KPSrecv.first;
    // POSERecv = math_.RotMat2Cartesian(KPSrecv.first);
    // cout << "POSERecv: " << POSERecv.tran_.x << POSERecv.tran_.y << POSERecv.tran_.z << POSERecv.rpy_.rx << POSERecv.rpy_.ry << POSERecv.rpy_.rz << endl;
    // cout << "qRecvseven_: " << KPSrecv.second << endl;
    POSE_start = POSERecv;
    LOG_DEBUG_ARM("POSE_start: {:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}",
                  POSE_start.tran_.x,
                  POSE_start.tran_.y,
                  POSE_start.tran_.z,
                  POSE_start.rpy_.rx,
                  POSE_start.rpy_.ry,
                  POSE_start.rpy_.rz);
    POS_now = POSE_start.tran_;
    RPY_now = POSE_start.rpy_;
    if (moveMode == MoveMode::abs) {
        POS_end = Targetpose.tran_;
        RPY_end = Targetpose.rpy_;
    } else if (moveMode == MoveMode::incr) {
        POS_end.x = POS_now.x + Targetpose.tran_.x;
        POS_end.y = POS_now.y + Targetpose.tran_.y;
        POS_end.z = POS_now.z + Targetpose.tran_.z;
        RPY_end.rx = RPY_now.rx + Targetpose.rpy_.rx;
        RPY_end.ry = RPY_now.ry + Targetpose.rpy_.ry;
        RPY_end.rz = RPY_now.rz + Targetpose.rpy_.rz;
    }
    // 在这里判断用户输入是否超限
    Pose target;
    target.tran_ = POS_end;
    target.rpy_ = RPY_end;
    auto TarIk = math_.IKine_seven(target, KPSrecv.second, qRecvseven_);
    vector<double> ik_temp;
    if (TarIk.first == true) {
        for (int i = 0; i < 7; i++) {
            ik_temp.push_back(TarIk.second(i));
        }
        auto TarCheck = math_.CheckAngle_seven(ik_temp, GetArmParam().axis.S_min_user, GetArmParam().axis.S_max_user);
        if (TarCheck == true) {
            Quat_now = math_.Rpy2quat(RPY_now);
            Quat_end = math_.Rpy2quat(RPY_end);
            Qs = math_.CumputeQs(Quat_now, Quat_end);

            S(0) = POS_end.x - POS_now.x;
            S(1) = POS_end.y - POS_now.y;
            S(2) = POS_end.z - POS_now.z;
            S(3) = Qs;
            S(4) = 0;  // 笛卡尔计算时间不需要，用0占位
            S(5) = 0;  // 笛卡尔计算时间不需要，用0占位
                       // cout << "S:" << S << endl;
            for (int i = 0; i < 6; i++) {
                if (fabs(S(i)) >= 1e-5) {
                    Needmove = true;  // 复合要求则会被置位为true
                }
            }
            // cout << "Needmove:" << Needmove << endl;
            if (Needmove == false) {
                // cout << "No need to move" << endl;
                T.first = RetState::ok;  // 如果无法移动，则返回noSupport
                moveData_.datas.clear();
            } else {  // 可以移动，进入计算部分
                T.second = ComputeT(S, Vuser, Auser, flag);
                // cout << "T:" << T << endl;
                Plan_T = T.second / dt_;
                // LOG_DEBUG_ARM("MoveL T: {:.4f},{}", T.second, dt_);
                for (double tpl = 0; tpl < Plan_T; tpl++) {
                    Ps = sqrt(pow(S(0), 2) + pow(S(1), 2) + pow(S(2), 2));

                    PY_ = math_.Plan5(Ps, Plan_T * dt_, tpl * dt_).Y;
                    QY_ = math_.Plan5(Qs, Plan_T * dt_, tpl * dt_).Y;

                    POSEend.x = POS_now.x + S(0) * (PY_ / Ps);
                    POSEend.y = POS_now.y + S(1) * (PY_ / Ps);
                    POSEend.z = POS_now.z + S(2) * (PY_ / Ps);

                    Quat = math_.QuatInterpolate(QY_, Qs, Quat_now, Quat_end);

                    ENDRPY = math_.Quat2rpy(Quat);

                    POSEND.tran_ = POSEend;
                    POSEND.rpy_ = ENDRPY;
                    // cout << "posend:" << endl;
                    auto ret = math_.IKine_seven(POSEND, KPSrecv.second, qRecvseven_);
                    if (ret.first == true) {
                        q_cmd = ret.second;

                        // cout << "q_cmd:" << q_cmd << endl;
                        for (int i = 0; i < 6; i++) {
                            qd_cmd(i) = (q_cmd(i) - qOldseven_(i)) / dt_;
                            qOldseven_(i) = q_cmd(i);
                        }
                        SaveAngle_seven(q_cmd);
                        // T.first = RetState::ok;
                    } else {
                        // cout << "IKine_seven failed" << endl;
                        moveData_.datas.clear();
                        T.first = RetState::outRange;
                    }
                }
                if (moveData_.datas.empty()) {
                    moveData_.datas.emplace_back(vector<msg::arm_cmd>{});
                }
                // cout << "sdaklfjlq:::::::" << planlist_.size() << endl;
                moveData_.datas.push_back(planlist_);
                T.first = RetState::ok;
            }
        } else {
            T.first = RetState::outRange;
        }
    } else {
        T.first = RetState::outRange;
    }
    return T;
}

MsgType ArmCore7::RxTxRxCallback(const MsgType& in)
{
    (void)in;
    Result<std::function<std::optional<msg::arm_cmd>(msg::arm_data)>> ret;
    ret.second = [this](msg::arm_data data) { return this->TxRxCallback(data); };
    ret.first = RetState::ok;
    return ret;
}
MsgType ArmCore7::RxInfoCallback(const MsgType& in)
{
    (void)in;
    Result<std::function<void(msg::arm_motor_info)>> ret;
    ret.second = [this](msg::arm_motor_info info) { return this->InfoCallback(info); };
    ret.first = RetState::ok;
    return ret;
}

// void ArmCore7::CalculateVelocity()
// {
//     Vec6<double> v, JointVel;
//     for (int i = 0; i < coData_->nowVelocity.size(); i++) {
//         JointVel(i) = coData_->nowVelocity[i];
//     }
//     msg::watch_data debug;
//     v = math_.Jacobian_tool(coData_->nowAngle) * JointVel;
//     for (int i = 0; i < 3; i++) {
//         debug.a[i] = v(i);  // 存放在a页的三个图表的蓝色线
//         // debug.ra[i] = ra(i);  // 存放在a页的三个图表的红色线
//         debug.b[i] = v(i + 3);  // 存放在b页的三个图表的蓝色线
//         // debug.rb[i] = rb(i);  // 存放在b页的三个图表的红色线
//         // debug.c[i] = c(i);  // 存放在c页的三个图表的蓝色线
//         // debug.rc[i] = rc(i);  // 存放在c页的三个图表的红色线
//     }
//     MsgTrySend("qr::debugData", debug);
//     // cout << "v:" << v << endl;
// }

/**
 * @brief 用户设置位置限制
 */
RetState ArmCore7::RxSetPositionLimit(const MsgType& in)
{
    RetState ret;
    auto set = in.GetType<vector<ValueRange<double>>>();
    for (int i = 0; i < 7; i++) {
        user_.Smax[i] = set[i].max;
        user_.Smin[i] = set[i].min;
    }

    ret = RetState::ok;

    return ret;
}

/**
 * @brief 用户设置加速度限制
 */
RetState ArmCore7::RxSetJointAccLimit(const MsgType& in)
{
    RetState ret;
    auto set = in.GetType<vector<double>>();
    for (int i = 0; i < 7; i++) {
        user_.Amax[i] = set[i];
    }

    ret = RetState::ok;
    return ret;
}
/**
 * @brief 用户设置速度限制
 */
RetState ArmCore7::RxSetJointSpeedLimit(const MsgType& in)
{
    RetState ret;
    auto set = in.GetType<vector<double>>();
    for (int i = 0; i < 7; i++) {
        user_.Vmax[i] = set[i];
    }
    ret = RetState::ok;

    return ret;
}
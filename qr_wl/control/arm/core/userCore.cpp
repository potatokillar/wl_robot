
#include "userCore.hpp"

#include "armParam.hpp"
#include "innerType.hpp"

using namespace std;
using namespace arm;

UserCore::UserCore(const BootArgs& args, const ArmBootArgs& armArgs) : boot_(args), dt_(armArgs.dt)
{
    // 均来自用户侧的指令，非阻塞
    if (GetArmParam().ArmAixsNum == 6) {
        cout << "arm6" << endl;
        armcore_ = make_unique<ArmCore6>(armArgs);
    } else if (GetArmParam().ArmAixsNum == 7) {
        cout << "arm7" << endl;
        armcore_ = make_unique<ArmCore7>(armArgs);
    } else {
        LOG_ERROR_ARM("ArmAixsNum is not 6 or 7");
    }

    thread_ = std::thread(&UserCore::Run, this);
}

/**
 * @description: 本函数周期并不是严格的4ms，因为里面存在RPC的发送，而RPC发送一定是阻塞的
 * @return {}
 */
void UserCore::Run()
{
    while (true) {
        armcore_->Run();
        TimerTools::SleepForMs(dt_ * 1000);
    }
}

/**
 * @description: 发送给qrChart
 * @return {}
 */
// void UserCore::Send2Server()
// {
//     Vec6<double> qData, ArmTeach;
//     Pose pos;

//     Pose ToolValue = user_.userTool[coData_->toolName];
//     pos = math_.Fkine(coData_->nowAngle, ToolValue);

//     ArmTeach << pos.tran_.x, pos.tran_.y, pos.tran_.z, pos.rpy_.rx, pos.rpy_.ry, pos.rpy_.rz;
//     MsgTrySend("arm::arm_teach_cmd", ArmTeach);
// }

/**
 * @description: 更新机械臂状态
 * @return {}
 */
// void UserCore::UpdateArmState()
// {
//     msg::arm_motor_state state = coData_->motorInfo.state;

//     // 根据电机状态，修改机械臂的状态
//     if (state == msg::arm_motor_state::disable) {
//         ClearAllMoveData();
//         SetArmState(ArmState::noReady);
//     } else if (state == msg::arm_motor_state::error) {
//         ClearAllMoveData();
//         SetArmState(ArmState::error);
//     } else if (state == msg::arm_motor_state::enable) {
//         if ((GetArmState() == ArmState::noReady) || (GetArmState() == ArmState::error)) {
//             SetArmState(ArmState::ready);
//         }
//     }
//     if (GetArmState() == ArmState::complete) {
//         if (TimerTools::GetNowTickMs() - lastStateChange_ > 100) {
//             SetArmState(ArmState::ready);
//         }
//     }
// }

/**
 * @description: 执行命令
 * RxUserCtrl函数是接收命令，本函数是具体执行，且关注执行结果
 * @return {}
 */
// void UserCore::ExecTaskCmd()
// {
//     if (moveData_.cmd.has_value()) {
//         auto cmd = moveData_.cmd.value();
//         if (GetArmState() == ArmState::noReady) {
//             LOG_WARN_ARM("Task noReady, can't exec {}", Enum2Num(cmd));
//             ReportTaskExecResult(RetState::noSupport);
//         } else if (GetArmState() == ArmState::error) {
//             LOG_WARN_ARM("Task error, can't exec {}", Enum2Num(cmd));
//             ReportTaskExecResult(RetState::error);
//         } else {
//             switch (cmd) {
//                 case ArmCtrl::pause: {
//                     SetArmState(ArmState::pause);
//                     LOG_INFO_ARM("Task pause");
//                     ReportTaskExecResult(RetState::interrupt);
//                 } break;
//                 case ArmCtrl::stop: {
//                     if (moveData_.idx < GetFirstMoveData().size()) {
//                         LOG_INFO_ARM("Task interrupt");
//                         SetArmState(ArmState::interrupt);
//                     } else {
//                         // 发送stop的时候刚好任务完成，概率太小，一般不会出现
//                         LOG_INFO_ARM("Task complete[stop]");
//                         SetArmState(ArmState::complete);
//                     }
//                     ClearAllMoveData();
//                     ReportTaskExecResult(RetState::interrupt);
//                 } break;

//                 case ArmCtrl::start:
//                 case ArmCtrl::resume: {
//                     // cout << "abc" << moveData_.idx << " " << moveData_.data.size() << endl;
//                     // cout << "cba" << Enum2Num(cmd) << endl;
//                     SetArmState(ArmState::running);
//                     if (moveData_.idx >= GetFirstMoveData().size()) {
//                         ClearFirstMoveData();
//                     }
//                     if (moveData_.datas.empty()) {
//                         SetArmState(ArmState::complete);
//                         LOG_INFO_ARM("Task complete");
//                         ReportTaskExecResult(RetState::ok);
//                     }
//                     // moveData_.cmd.reset();  // 执行完成但不上报状态，todo任务状态是否需要独立的状态上报
//                 } break;

//                 default: {
//                     LOG_WARN_ARM("exec cmd no support");
//                     ReportTaskExecResult(RetState::noSupport);
//                 } break;
//             }
//         }
//     }
// }

/**
 * @description: 上报任务执行结果
 * @param in
 * @return {}
 */
// void UserCore::ReportTaskExecResult(RetState ret)
// {
//     moveData_.cmd.reset();  // 发送task_exec之前，必须清除cmd
//     MsgTrySend("arm::task_exec", ret);
// }

/**
 * @description: 接收用户侧的设置IO指令
 * @param in
 * @return {}
 */
// MsgType UserCore::RxSetIoOut(const MsgType& in)
// {
//     auto ioCtl = in.GetType<std::pair<string, std::bitset<32>>>();
//     // motor_->SetIoOut(ioCtl.first, ioCtl.second);
//     MsgTrySend("arm::set_io", ioCtl);
//     return RetState::ok;  // 必定成功
// }

/**
 * @description: 接收来自用户侧的控制指令
 * @param &in
 * @return {} 必成功
 */
// MsgType UserCore::RxUserCtrl(const MsgType& in)
// {
//     auto cmd = in.GetType<ArmCtrl>();
//     switch (cmd) {
//         case ArmCtrl::enable:
//             // LOG_INFO_ARM("enable motor");
//             // motor_->Enable();
//             MsgTrySend("arm::enable", true);
//             break;
//         case ArmCtrl::disable:
//         case ArmCtrl::emgStop:
//             // LOG_INFO_ARM("disable motor");
//             // motor_->Disable();
//             MsgTrySend("arm::enable", false);
//             break;
//         default:
//             // 必须前一个命令执行完毕
//             //  if (moveData_.cmd.has_value() == false) {
//             moveData_.cmd = cmd;  // 无需上一个命令
//             //  } else {
//             //      LOG_WARN_ARM("last cmd no exec complete");
//             //     return RetState::busy;
//             //  }

//             break;
//     }
//     // 需要删除之前的arm::task_exec上报，因为部分情况下，RxWaitTaskComplete不会调用。
//     // todo msg接口临时方案，等待msg优化后再次修改
//     MsgSend msg("arm::task_exec");
//     msg.Clear();
//     return RetState::ok;
// }

/**
 * @description: 接收来自用户侧的请求
 * @param in
 * @return {}
 */
// MsgType UserCore::RxGetToolValue(const MsgType& in)
// {
//     string toolName = in.GetType<string>();
//     Pose ToolValue;
//     const auto& tool = user_.userTool;
//     auto it = tool.find(toolName);
//     if (it != tool.end()) {
//         ToolValue = it->second;
//     }
//     return ToolValue;
// }

// MsgType UserCore::RxGetToolNameList(const MsgType& in)
// {
//     (void)in;

//     vector<string> UserToolName;
//     for (const auto [num, tool] : user_.userTool) {
//         UserToolName.push_back(num);
//     }
//     return UserToolName;
// }

// MsgType UserCore::RxSetToolName(const MsgType& in)
// {
//     RetState ret;
//     auto toolName = in.GetType<string>();

//     if (user_.userTool.count(toolName) == 1) {
//         coData_->toolName = toolName;
//         ret = RetState::ok;
//     } else {
//         ret = RetState::noExist;
//     }

//     return ret;
// }
// MsgType UserCore::RxGetToolName(const MsgType& in)
// {
//     (void)in;
//     return coData_->toolName;
// }

/**
 * @description: 机械臂参数读取，和pub_info的区别是，这种信息是固定不会实时改变的
 * @param in
 * @return {}
 * @todo Vec6改为六轴七轴通用的vector
 */

// void UserCore::PubInfo()
// {
//     ArmUserInfo info;
//     info.angle.clear();
//     for (int i = 0; i < 6; i++) {
//         info.angle.push_back(coData_->nowAngle(i));  //.push_back();
//     }
//     Pose ToolValue = user_.userTool[coData_->toolName];

//     if (GetArmParam().ArmAixsNum == 6) {
//         info.cart = math_.Fkine(coData_->nowAngle, ToolValue);
//     }  // else if (GetArmParam().ArmAixsNum == 7) {
//     //     Vec7<double> now;
//     //     std::pair<Pose, double> EndPose;
//     //     for (int i = 0; i < 7; i++) {
//     //         now(i) = coData_->nowAngle[i];
//     //     }
//     //     EndPose = math_.Fkine_seven(now, ToolValue);
//     //     info.cart = EndPose.first;
//     //     info.Phi = EndPose.second;
//     // }

//     info.state = coData_->armSta;
//     info.ioInSta = coData_->motorInfo.ioInSta;
//     info.ioOutSta = coData_->motorInfo.ioOutSta;
//     // info.motor.current = coData_->motorInfo.errCode; // todo电压电流数据未实现

//     MsgTrySend("arm::pub_info", info);
// }

/**
 * @description: 发送接收回调，由motor线程调用
 * @param ret
 * @todo Vec6改为六轴七轴通用的vector
 * @return {}
 */
// std::optional<msg::arm_cmd> UserCore::TxRxCallback(const msg::arm_data& ret)
// {
//     lock_guard lock(mtx_);
//     //////////接收处理///////////////////
//     if (GetArmParam().ArmAixsNum == 6) {
//         for (int i = 0; i < 6; i++) {
//             coData_->nowAngle(i) = ret.motor[i].alpha * GetArmParam().axis.gear_flip[i];  // real
//             coData_->nowVelocity(i) = ret.motor[i].torq * GetArmParam().axis.gear_flip[i];
//         }
//     }

//     /////////发送处理////////////////////////
//     // running状态才能发送
//     if (GetArmState() != ArmState::running) {
//         return std::nullopt;
//     }

//     // 空容器不发送
//     if (moveData_.datas.empty()) {
//         return std::nullopt;
//     }

//     msg::arm_cmd cmd;
//     if (moveData_.idx < GetFirstMoveData().size()) {
//         vector<double> qcmd;
//         qcmd.clear();
//         for (int i = 0; i < 6; i++) {
//             qcmd.push_back(GetFirstMoveData()[moveData_.idx].motor[i].alpha);
//         }

//         // cout << " cmd:" << qcmd << endl;
//         bool ret = true;
//         // ret = math_.CheckAngle(qcmd, GetArmParam().axis.S_min_real, GetArmParam().axis.S_max_real);
//         if (ret == false) {
//             ClearFirstMoveData();
//             LOG_WARN_ARM("CheckAngle fail, q > qLimit");
//             SetArmState(ArmState::error);
//             return std::nullopt;
//         } else {
//             for (int slc = 1; slc <= 6; slc++) {
//                 cmd.motor[slc - 1].alpha = qcmd[slc - 1];
//             }
//             moveData_.idx++;
//         }

//         return cmd;
//     }
//     return std::nullopt;
// }

/**
 * @description: motor其他信息上报回调
 * @param info
 * @return {}
 */
// void UserCore::InfoCallback(const msg::arm_motor_info& info)
// {
//     lock_guard lock(mtx_);
//     coData_->motorInfo = info;
//     // LOG_DEBUG_ARM("rx motor state: {}", Enum2Num(info.state));
// }

/**
 * @description: 接收阻塞任务
 * @param in
 * @todo 关节规划根据数据长度判断，笛卡尔规划改为根据型号判断
 * @return {}
 */
// MsgType UserCore::RxMoveSearch(const MsgType& in) { return armcore_->MoveSearch(in); }

// RetState UserCore::RxSetRapidRate(const MsgType& in)
// {
//     RetState ret;
//     auto set = in.GetType<double>();

//     ret = math_.Checkrapid(set);
//     if (ret == RetState::ok) {
//         user_.rapidrate = set;
//     } else {
//         user_.rapidrate = 0.5;
//     }
//     return ret;
// }
/**
 * @brief 用户设置位置限制
 */
// RetState UserCore::RxSetPositionLimit(const MsgType& in)
// {
//     RetState ret;
//     bool state;
//     auto set = in.GetType<vector<ValueRange<double>>>();
//     for (int i = 0; i < 6; i++) {
//         user_.Smax[i] = set[i].max;
//         user_.Smin[i] = set[i].min;
//     }

//     if (set.size() == 6) {
//         // state = sql_.SyncWrite("arm", "6_PositionLimit", "vector<ValueRange<double>>", set);
//     }
//     if (state == true) {
//         ret = RetState::ok;
//     } else {
//         ret = RetState::error;
//     }
//     return ret;
// }

// Result<ValueRange<vector<double>>> UserCore::RxGetPositionLimit()
// {
//     Result<ValueRange<vector<double>>> ret;
//     auto data = sql_.Read("arm", "6_PositionLimit");
//     if (data.first == true) {
//         auto value = data.second;
//         ValueRange<vector<double>> set;
//         std::stringstream ss(value);
//         std::string item;
//         vector<double> v;
//         while (std::getline(ss, item, ' ')) {
//             v.push_back(std::stod(item));
//         }
//         for (size_t i = 0; i < v.size(); i++) {
//             if (i % 2 == 0) {
//                 set.min.push_back(v[i]);
//             } else {
//                 set.max.push_back(v[i]);
//             }
//         }
//         // for(int i = 0; i < set.max.size(); i++)
//         // {
//         //     cout << set.max[i] << " " << set.min[i] << endl;
//         // }
//         ret.first = RetState::ok;
//         ret.second = set;
//     }

//     return ret;
// }

/**
 * @brief 用户设置加速度限制
 */
// RetState UserCore::RxSetJointAccLimit(const MsgType& in)
// {
//     RetState ret;
//     std::string value;
//     auto set = in.GetType<pair<string, Pose>>();
//     value.append(std::to_string(set.second.tran_.x));
//     value.append(" ");
//     value.append(std::to_string(set.second.tran_.y));
//     value.append(" ");
//     value.append(std::to_string(set.second.tran_.z));
//     value.append(" ");
//     value.append(std::to_string(set.second.rpy_.rx));
//     value.append(" ");
//     value.append(std::to_string(set.second.rpy_.ry));
//     value.append(" ");
//     value.append(std::to_string(set.second.rpy_.rz));
//     value.append(" ");
//     bool state = sql_.SyncWrite("arm", "UserTool_" + set.first, "Pose", value);
//     if (state == true) {
//         ret = RetState::ok;
//     } else {
//         ret = RetState::error;
//     }

//     ret = RetState::ok;
//     return ret;
// }
/**
 * @brief 用户设置速度限制
 */
// RetState UserCore::RxSetJointSpeedLimit(const MsgType& in)
// {
//     RetState ret;
//     auto set = in.GetType<vector<double>>();
//     for (int i = 0; i < 6; i++) {
//         user_.Vmax[i] = set[i];
//     }
//     ret = RetState::ok;

//     return ret;
// }
// RetState UserCore::RxSetToolValue(const MsgType& in)
// {
//     map<std::string, Pose> userset;

//     auto set = in.GetType<ArmUserTool>();
//     // userset.insert(pair<std::string, Pose>(set.toolName, set.toolPose));
//     // user_.userTool.emplace(set.toolName, set.toolPose);
//     user_.userTool[set.toolName] = set.toolPose;
//     return RetState::ok;
// }
// Result<ValueRange<Vec6<double>>> UserCore::RxGetPositionLimit() {}

// RetState UserCore::RxSetPositionLimit(const MsgType& in)
// {
//     RetState ret;
//     bool state = false;
//     auto set = in.GetType<vector<ValueRange<double>>>();
//     std::string value;
//     for (auto v : set) {
//         value.append(std::to_string(v.min));
//         value.append(" ");
//         value.append(std::to_string(v.max));
//         value.append(" ");
//     }
//     if (set.size() == 6) {
//         state = sql_.UpdateOrInsert("arm", "6_PositionLimit", "vector<double>", value);
//     }
//     if (state == true) {
//         ret = RetState::ok;
//     } else {
//         ret = RetState::error;
//     }
//     return ret;
// }

// Result<ValueRange<vector<double>>> UserCore::RxGetPositionLimit()
// {
//     Result<ValueRange<vector<double>>> ret;
//     auto data = sql_.Select("arm", "6_PositionLimit");
//     if (data.first == true) {
//         auto value = data.second;
//         ValueRange<vector<double>> set;
//         std::stringstream ss(value);
//         std::string item;
//         vector<double> v;
//         while (std::getline(ss, item, ' ')) {
//             v.push_back(std::stod(item));
//         }
//         for (size_t i = 0; i < v.size(); i++) {
//             if (i % 2 == 0) {
//                 set.min.push_back(v[i]);
//             } else {
//                 set.max.push_back(v[i]);
//             }
//         }
//         // for(int i = 0; i < set.max.size(); i++)
//         // {
//         //     cout << set.max[i] << " " << set.min[i] << endl;
//         // }
//         ret.first = RetState::ok;
//         ret.second = set;
//     }

//     return ret;
// }

// RetState UserCore::RxSetUserTool(const MsgType& in)
// {
//     RetState ret;
//     std::string value;
//     auto set = in.GetType<pair<string, Pose>>();
//     value.append(std::to_string(set.second.tran_.x));
//     value.append(" ");
//     value.append(std::to_string(set.second.tran_.y));
//     value.append(" ");
//     value.append(std::to_string(set.second.tran_.z));
//     value.append(" ");
//     value.append(std::to_string(set.second.rpy_.rx));
//     value.append(" ");
//     value.append(std::to_string(set.second.rpy_.ry));
//     value.append(" ");
//     value.append(std::to_string(set.second.rpy_.rz));
//     value.append(" ");
//     bool state = sql_.UpdateOrInsert("arm", "UserTool_" + set.first, "Pose", value);
//     if (state == true) {
//         ret = RetState::ok;
//     } else {
//         ret = RetState::error;
//     }
//     return ret;
// }

// Result<Pose> UserCore::RxGetUserTool(const MsgType& in)
// {
//     Result<Pose> ret;
//     auto set = in.GetType<string>();
//     auto data = sql_.Read("arm", "UserTool_" + set);
//     if (data.first == true) {
//         auto value = data.second;
//         std::stringstream ss(value);
//         std::string item;
//         vector<double> v;
//     }
//     return ret;
// }

// MsgType UserCore::RxTxRxCallback(const MsgType& in)
// {
//     (void)in;
//     Result<std::function<std::optional<msg::arm_cmd>(msg::arm_data)>> ret;
//     ret.second = [this](msg::arm_data data) { return this->TxRxCallback(data); };
//     ret.first = RetState::ok;
//     return ret;
// }
// MsgType UserCore::RxInfoCallback(const MsgType& in)
// {
//     (void)in;
//     Result<std::function<void(msg::arm_motor_info)>> ret;
//     ret.second = [this](msg::arm_motor_info info) { return this->InfoCallback(info); };
//     ret.first = RetState::ok;
//     return ret;
// }

/**
 * @description: 清楚数据
 * @return {}
 */
// void UserCore::ClearFirstMoveData()
// {
//     if (!moveData_.datas.empty()) {
//         moveData_.datas.pop_front();
//         moveData_.idx = 0;
//     }
// }
// void UserCore::ClearLastMoveData()
// {
//     if (!moveData_.datas.empty()) {
//         moveData_.datas.pop_back();  // 这个不会影响idx}
//     }
// }
// void UserCore::ClearAllMoveData()
// {
//     moveData_.datas.clear();
//     moveData_.idx = 0;
// }

// /**
//  * @description: 获取第一个数据的索引
//  * @param moveData_
//  * @return {}
//  */
// // todo 空会导致未定义，需修改
// const std::vector<msg::arm_cmd>& UserCore::GetFirstMoveData() { return moveData_.datas.front(); }

// void UserCore::CalculateVelocity()
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

// /**
//  * @brief 离线规划数据记录
//  *
//  * @param q 关节角度
//  * @param qd 关节角速度
//  */
// void UserCore::SaveAngle(Vec6<double> q, Vec6<double> qd)
// {
//     (void)qd;
//     msg::arm_cmd cmd;

//     for (int i = 0; i < 6; i++) {
//         cmd.motor[i].alpha = q(i) * GetArmParam().axis.gear_flip(i);
//     }
//     // cmd.motor[1].alpha = -q(1);
//     // cmd.motor[4].alpha = -q(4);
//     planlist_.push_back(cmd);
// }

// /**
//  * @brief 计算规划中最长的时间T
//  * 存在部分错误，计算时间可能不准确
//  */
// double UserCore::ComputeT(Vec6<double> S, Vec6<double> V, Vec6<double> Amax, MoveFlag flag)
// {
//     Vec6<double> tmp1, tmp2, vlim, Ta, Td, Tv, T;
//     double Tzy = 0, tmpT = 0, zy = 7;

//     for (int i = 0; i < 6; i++) {
//         S(i) = fabs(S(i));
//         tmp1(i) = std::pow(V(i), 2);
//         tmp2(i) = S(i) * Amax(i);
//         if (tmp1(i) <= tmp2(i)) {
//             vlim(i) = V(i);
//             Ta(i) = vlim(i) / Amax(i);
//             Tv(i) = S(i) - (std::pow(vlim(i), 2) / Amax(i));
//             T(i) = 2 * Ta(i) + Tv(i);
//         } else {
//             vlim(i) = sqrt(S(i) * Amax(i));
//             Ta(i) = vlim(i) / Amax(i);
//             T(i) = 2 * Ta(i);
//         }
//     }
//     // cout << "S:" << S << endl;
//     // cout << "T_:" << T << endl;
//     tmpT = T(0);
//     if (flag == MoveFlag::movej) {
//         for (int i = 0; i < 6; i++) {
//             if (T(i) >= tmpT) {
//                 tmpT = T(i);
//                 zy = i;
//             }
//         }
//     } else if (flag == MoveFlag::movel) {
//         for (int i = 0; i < 4; i++) {
//             if (T(i) >= tmpT) {
//                 tmpT = T(i);
//                 zy = i;
//             }
//         }
//     }
//     if (zy == 7) {
//         Tzy = 0;
//     } else {
//         Tzy = T(zy);
//     }
//     if (Tzy < 0.0001) {
//         Tzy = 0.04;
//     }
//     return Tzy;
// }
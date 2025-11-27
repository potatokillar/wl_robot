
#pragma once
#include <onnx/onnxruntime_cxx_api.h>

#include "baseline.hpp"
#include "contrType.hpp"
#include "fsmState.hpp"
#include "imuData.hpp"
#include "innerType.hpp"
#include "jpCtrlData.hpp"
#include "mathTools.hpp"

class FsmHandStand : public FsmState
{
public:
    FsmHandStand(ContrContainer contr, std::shared_ptr<JpData> jpData);
    void OnEnter() override;
    void Run() override;
    void OnExit() override;

    void LeggedRun();
    void LeggedWheelRun();
    void NetWorkResult(std::vector<double> imu, float vx, float vy, float wz, DataCmd motorRet);
    void NetWorkResult(std::vector<double> imu, float vx, float vy, float wz, CmdVal4 wheelMotorRet);
    DataCmd SetLeggedPID();
    CmdVal4 SetLeggedWheelPID();
    void PrintNetworkResult(std::vector<float> out_cmd_val);

private:
    // TODO: sort the vars below
    u64 interval_{0};
    ImuData imu_;
    DataCmd motorRet_;

    std::string onnxPath_;

    std::vector<double> transXYZ(double x, double y, double z, double w);

    ////////onnx///////////
    std::unique_ptr<Ort::Session> session_;
    int64_t input_tensor_size_;

    std::vector<float> actionsOut_;
    std::vector<float> lastActions_;
    std::vector<float> sendActions_;
    std::vector<float> lastSendActions_;

    // mutable std::mutex aiMutex_;
    std::vector<double> defaultJoint_;
    double aiActionScale_{0.25};
    double kFilter_{0.3};
    bool printInfo_{false};

    float vx_{0}, vy_{0}, wz_{0};
    CmdVal2 aiSend_;
    Mat34<double> EnterQ_;
    Mat34<double> qStandForAI0, qStandForAI1, qStandForAI2;

    CmdVal4 wheelMotorRet_;
    CmdVal4 wheelMotorCmdOld;
    DataValue2 wheelAiSend_;
    Mat44<double> wheelStandQ_;

    // int N = 3;
    int iRun_ = 0;
    double kSmooth = 0.0;
    DataCmd motorCmdOld;
    CmdVal4 motorCmdWheelOld;

    bool isWheel_;
    double vxTemp = 0;
    double vyTemp = 0;
    double wzTemp = 0;

    const QuadrupedParam& qrParam{GetQrParam()};
    MathFun mathFun_;
};
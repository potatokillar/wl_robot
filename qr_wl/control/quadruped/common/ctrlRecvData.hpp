

#pragma once

#include <deque>
#include <memory>

#include "Transformer.hpp"
#include "baseline.hpp"
#include "imuData.hpp"
#include "innerType.hpp"
#include "quadrupedParam.hpp"
#include "stateEstimatorContainer.hpp"

class CtrlRecvData
{
public:
    CtrlRecvData(bool fromSim);

    void LoadData();

    const ImuData& GetImuData() const { return imuData_; }
    const DataCmd& GetData3() const { return motorData_; }
    const DataCmd& GetMotorDataMit() const { return motorData2_; }
    const CmdVal4& GetMotorDataWheel() const { return motorDataWheel_; }
    const CmdVal4& GetMotorDataWheelMit() const { return motorDataWheel2_; }

    void SetFromSim(bool set) { fromSim_ = set; }

private:
    void Deal();
    DataCmd DealMotionData(const msg::qr::motor_ret& rawdata, DataCmd& oldData);
    DataCmd CheckData(const DataCmd& motorData, const DataCmd& motorDataOld);
    void CheckData(CmdVal4* motorData, CmdVal4* motorDataOld);
    void DealLegWheelMotorData(const msg::wheel::motor_ret& rawData);
    CmdVal4 MotorDataUnitree2Algo(const CmdVal4 &wheelMotorDataUnitree);
    DataCmd MotorDataReal2Unitree(const DataCmd &motorCmdReal);

private:
    bool fromSim_{true};

    msg::imu_data rawImuData_;
    msg::qr::motor_ret rawMotionData_;
    msg::wheel::motor_ret rawMotorWheelData_;

    DataCmd motorData_;
    DataCmd motorData3_;
    DataCmd motorData2_;

    CmdVal4 motorDataWheel3_;
    CmdVal4 motorDataWheel_;
    CmdVal4 motorDataWheel2_;
    MathFun mathFun_;
    ImuData imuData_;

    TransformCoordinate transform_;

    const QuadrupedParam& qrParam{GetQrParam()};
};
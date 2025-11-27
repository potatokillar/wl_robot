
#pragma once
#include <array>
#include <memory>
#include <vector>

#include "apiArmCtrl.hpp"
#include "armTask.hpp"
#include "baseline.hpp"

class ArmNode final
{
public:
    static ArmNode& GetInstance()
    {
        static ArmNode instance;
        return instance;
    }

    void Start(const BootArgs& args, const ArmBootArgs& armArgs);
    void Stop();
    void Run();

private:
    std::unique_ptr<ArmTask> armTask_;
    std::unique_ptr<PeriodicMemberFunction<ArmNode>> task_;
};

inline ArmNode& GetArmNode() { return ArmNode::GetInstance(); }
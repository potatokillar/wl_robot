
#pragma once

#include "qrTask.hpp"
#include "stateManager.hpp"

class CtrlNode final
{
public:
    static CtrlNode& GetInstance()
    {
        static CtrlNode instance;
        return instance;
    }

    void Start(const BootArgs& args, const QrBootArgs& qrArgs);
    void Stop();

private:
    CtrlNode() {};
    std::unique_ptr<StateManager> staManager_;
};
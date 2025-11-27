
#pragma once

#include <alsa/asoundlib.h>

#include <string>

#include "deviceParam.hpp"
#include "pcmDrv.hpp"

class MediaPlay
{
public:
    MediaPlay() {};
    virtual ~MediaPlay() {};

    virtual void Play() = 0;
    virtual void Pause() = 0;
    virtual void Stop() = 0;
    virtual std::string GetFullName() = 0;
    virtual bool IsNumberAndUnit() = 0;
    virtual std::string GetName() = 0;

private:
};

class WavPlay : public MediaPlay
{
public:
    WavPlay(const std::string &pathname);
    WavPlay(const std::string &pathname, const std::string &name);
    virtual ~WavPlay();
    void Play() override;
    void Stop() override {};
    void Pause() override {};

    std::string GetFullName() override;
    bool IsNumberAndUnit() override;

    std::string GetName() { return name_; };

private:
    std::string name_ = {};
    bool initOk = false;
    std::string fullname;
    PcmDrv pcm_;
};

#pragma once

#include <alsa/asoundlib.h>

#include "baseline.hpp"

struct WavHeader
{
    s8 riff[4];
    u32 fileLen;
    s8 wave[4];
    s8 fmt[4];

    u32 formatLen;
    u16 formatTag;
    u16 Channels;
    u32 SampleRate;
    u32 AvgBitsPerSample;
    u16 BlockAlign;
    u16 BitsPerSample;
};

class PcmDrv
{
public:
    bool Open(const std::string &devName);
    void Close();
    bool SetParam(const WavHeader &wav_header);
    void Play(FILE *fp);

    bool isOpen = false;

private:
    snd_pcm_t *handle = NULL;
    snd_pcm_uframes_t period_size;
    int bufSize = 0;
    bool DevIsExist(const std::string &devName);
};
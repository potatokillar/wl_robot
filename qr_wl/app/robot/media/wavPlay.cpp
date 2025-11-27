
#include "wavPlay.hpp"

#include <alsa/asoundlib.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "baseline.hpp"
#include "pcmDrv.hpp"

using namespace ::std;
const std::vector<std::string> skipList_ = {"0", "1", "2", "3", "4", "5", "6", "7", "8", "9", "kg", "point"};

WavPlay::WavPlay(const std::string &pathname) { fullname = pathname; }

WavPlay::WavPlay(const std::string &pathname, const std::string &name)
{
    fullname = pathname;
    name_ = name;
};

void WavPlay::Play()
{
    // 打开音乐文件
    FILE *fp = fopen(fullname.c_str(), "rb");
    if (fp == NULL) {
        LOG_ERROR("music file is NULL");
        // fclose(fp);
    }

    WavHeader wav_header;
    if (fread(&wav_header, 1, sizeof(wav_header), fp) <= 0) {
        LOG_ERROR("music file is uncorrent!");
        fclose(fp);
        return;
    };

    // LOG_INFO("play music {}", fullname);

    pcm_.Open(GetDevParam().audioName);
    pcm_.SetParam(wav_header);
    TimerTools::SleepForMs(10);
    pcm_.Play(fp);
    pcm_.Close();

    fclose(fp);
}

WavPlay::~WavPlay() {}

std::string WavPlay::GetFullName() { return fullname; }
bool WavPlay::IsNumberAndUnit() { return (std::find(skipList_.begin(), skipList_.end(), name_) != skipList_.end()); };
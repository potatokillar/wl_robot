#include "pcmDrv.hpp"

#include <fstream>

#include "baseline.hpp"

using namespace ::std;

bool PcmDrv::DevIsExist(const std::string &devName)
{
    std::ifstream ifs("/proc/asound/cards");
    std::string line;

    if (ifs.is_open() == false) {
        return false;
    }

    auto cardName = devName.substr(3);

    while (getline(ifs, line)) {
        if (line.empty() == true) {
            continue;
        }

        auto n = line.find("[" + cardName);
        if (n != string::npos) {
            auto next = line.substr(n + cardName.size() + 1, 1);
            if ((next == " ") || (next == "]")) {
                return true;
            }
        }
    }
    return false;
}

bool PcmDrv::Open(const std::string &devName)
{
    if (isOpen == true) {
        return true;
    }

    if (DevIsExist(devName) != true) {
        LOG_WARN("can't find audio dev: {}", devName);
        return false;
    }

    int ret = snd_pcm_open(&handle, devName.c_str(), SND_PCM_STREAM_PLAYBACK, 0);
    if (ret < 0) {
        LOG_ERROR("open PCM device failed");
        handle = NULL;
        return false;
    }

    isOpen = true;
    return true;
}

void PcmDrv::Close()
{
    if (isOpen == false) {
        return;
    }

    /* pass the remaining samples, otherwise they're dropped in close */
    int err = snd_pcm_drain(handle);
    if (err < 0) {
        LOG_WARN("snd_pcm_drain failed: {}", snd_strerror(err));
    }
    snd_pcm_close(handle);
    handle = NULL;
    isOpen = false;
}

bool PcmDrv::SetParam(const WavHeader &wav_header)
{
    if (isOpen == false) {
        return false;
    }

    int err, dir;
    unsigned int rrate;
    snd_pcm_access_t access = SND_PCM_ACCESS_RW_INTERLEAVED;

    snd_pcm_hw_params_t *hwparams;
    snd_pcm_hw_params_alloca(&hwparams);  // 分配params结构体

    /* choose all parameters */
    err = snd_pcm_hw_params_any(handle, hwparams);
    if (err < 0) {
        LOG_ERROR("Broken configuration for playback: no configurations available: {}", snd_strerror(err));
        return false;
    }
#if 0
    /* set hardware resampling */
    unsigned int resample = 1;
    err = snd_pcm_hw_params_set_rate_resample(handle, hwparams, resample);
    if (err < 0) {
        LOG_ERROR("Resampling setup failed for playback: {}", snd_strerror(err));
        return err;
    }
#endif
    /* set the interleaved read/write format */
    err = snd_pcm_hw_params_set_access(handle, hwparams, access);
    if (err < 0) {
        LOG_ERROR("Access type not available for playback: {}", snd_strerror(err));
        return false;
    }
    /* set the sample format */
    snd_pcm_format_t format = SND_PCM_FORMAT_S16_LE;
    switch (wav_header.BitsPerSample / 8) {
        case 1:
            format = SND_PCM_FORMAT_U8;
            break;
        case 2:
            format = SND_PCM_FORMAT_S16_LE;
            break;
        case 3:
            format = SND_PCM_FORMAT_S24_LE;
            break;
    }
    err = snd_pcm_hw_params_set_format(handle, hwparams, format);
    if (err < 0) {
        LOG_ERROR("Sample format not available for playback: {}", snd_strerror(err));
        return false;
    }
    /* set the count of channels */
    unsigned int channels = wav_header.Channels;
    err = snd_pcm_hw_params_set_channels(handle, hwparams, channels);
    if (err < 0) {
        LOG_ERROR("Channels count ({}) not available for playbacks: {}", channels, snd_strerror(err));
        return false;
    }
    /* set the stream rate */
    unsigned int rate = wav_header.SampleRate;
    rrate = rate;
    err = snd_pcm_hw_params_set_rate_near(handle, hwparams, &rrate, 0);
    if (err < 0) {
        LOG_ERROR("Rate %uHz not available for playback: {}", rate, snd_strerror(err));
        return false;
    }
    if (rrate != rate) {
        LOG_ERROR("Rate doesn't match (requested {}Hz, get {}Hz)\n", rate, err);
        return false;
    }
#if 0
    /* set the buffer time */
    err = snd_pcm_hw_params_set_buffer_time_near(handle, hwparams, &buffer_time, &dir);
    if (err < 0) {
        printf("Unable to set buffer time %u for playback: %s\n", buffer_time, snd_strerror(err));
        return err;
    }
    err = snd_pcm_hw_params_get_buffer_size(hwparams, &size);
    if (err < 0) {
        printf("Unable to get buffer size for playback: %s\n", snd_strerror(err));
        return err;
    }
    buffer_size = size;

    /* set the period time */
    err = snd_pcm_hw_params_set_period_time_near(handle, hwparams, &period_time, &dir);
    if (err < 0) {
        printf("Unable to set period time %u for playback: %s\n", period_time, snd_strerror(err));
        return err;
    }
#endif
    /* write the parameters to device */
    err = snd_pcm_hw_params(handle, hwparams);
    if (err < 0) {
        LOG_ERROR("Unable to set hw params for playback: {}", snd_strerror(err));
        return false;
    }

    err = snd_pcm_hw_params_get_period_size(hwparams, &period_size, &dir);
    if (err < 0) {
        LOG_ERROR("Unable to get period size for playback: {}", snd_strerror(err));
        return false;
    }
    bufSize = period_size * wav_header.BlockAlign; /*4 代表数据快长度*/

    return true;
}

void PcmDrv::Play(FILE *fp)
{
    if ((bufSize == 0) || (fp == NULL) || (isOpen == false)) {
        return;
    }
    fseek(fp, 78, SEEK_SET);
    char *buffer = (char *)malloc(bufSize);

    while (1) {
        memset(buffer, 0, bufSize);
        int ret = fread(buffer, 1, bufSize, fp);
        if (ret == 0) {
            // printf("end of music file input!\n");
            return;
        }

        while ((ret = snd_pcm_writei(handle, buffer, period_size)) < 0) {
            // usleep(2000);
            if (ret == -EPIPE) {
                LOG_ERROR("underrun occurred -32, err_info = {}", snd_strerror(ret));

                snd_pcm_prepare(handle);
            } else if (ret < 0) {
                LOG_ERROR("error from writei: {}", snd_strerror(ret));
            }
        }
    }

    free(buffer);
}
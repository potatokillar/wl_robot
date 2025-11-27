
#include "robotMedia.hpp"

#include <filesystem>

#include "mediaConfig.hpp"
#include "pcmDrv.hpp"

using namespace std;
namespace fs = std::filesystem;

static MediaTaskPackage default_media_task_pkg = MediaTaskPackage();

MediaTaskPackage::MediaTaskPackage(const std::string &name) { Add(name); }

void MediaTaskPackage::Add(const std::string &name)
{
    string pathname;
    string name_;

    auto language = GetRobotConfigDirect<string>("language");
    if (language) {
        if (language.value() == "zh-cn") {
            for (size_t i = 0; i < sizeof(mediaFile) / sizeof(MediaFile); i++) {
                if (mediaFile[i].name == name) {
                    pathname = mediaFile[i].zh_cn;
                    name_ = name;
                }
            }

        } else if (language.value() == "en-us") {
            for (size_t i = 0; i < sizeof(mediaFile) / sizeof(MediaFile); i++) {
                if (mediaFile[i].name == name) {
                    pathname = mediaFile[i].en_us;
                    name_ = name;
                }
            }
        }
    }

    try {
        fs::path absolutePath = fs::absolute(pathname);
        if (fs::exists(absolutePath)) {
            if (absolutePath.extension() == ".wav") {
                auto media = std::make_shared<WavPlay>(pathname, name_);
                _media_list.push_back(media);
            }
        } else {
            LOG_ERROR("can not find audio file: {}", pathname);
        }
    } catch (std::exception &) {
        LOG_ERROR("exception! can not absolute audio file: {}", pathname);
    }
}

bool MediaTaskPackage::operator==(const MediaTaskPackage &other) const
{
    if (_media_list.size() == 0 && other._media_list.size() == 0) {
        return true;
    }

    if (_media_list.size() != 0 && other._media_list.size() != 0) {
        int a = 0, b = 0;
        for (size_t i = 0; i < _media_list.size(); ++i) {
            if (_media_list[i]->IsNumberAndUnit()) {
                a = i;
            }
        }
        for (size_t i = 0; i < other._media_list.size(); ++i) {
            if (other._media_list[i]->IsNumberAndUnit()) {
                b = i;
            }
        }

        if (_media_list[a]->GetFullName() == other._media_list[b]->GetFullName()) {
            return true;
        }
    }

    return false;
}

bool MediaTaskPackage::operator!=(const MediaTaskPackage &other) const { return !(*this == other); }

RobotMedia::~RobotMedia() {}

void RobotMedia::AddPlay(std::shared_ptr<MediaPlay> mediaPlay)
{
    std::unique_lock<std::mutex> lock(_mutex);
    _readyPlay.push_back(mediaPlay);
}

void RobotMedia::AddPackage(const MediaTaskPackage &pkg)
{
    std::unique_lock<std::mutex> lock(_mutex);
    _package_list.push_back(pkg);
}

void RobotMedia::Unpack()
{
    if (_package_list.empty()) {
        return;
    }

    std::deque<MediaTaskPackage>::reverse_iterator rit = _package_list.rbegin();
    bool skip_first_match = false;
    while (rit != _package_list.rend()) {
        if (*rit == cache_) {
            if (skip_first_match == false) {
                ++rit;
                skip_first_match = true;
            } else {
                auto it = rit.base();
                --it;
                it = _package_list.erase(it);
                rit = std::deque<MediaTaskPackage>::reverse_iterator(it);
            }
        } else {
            ++rit;
        }
    }

    if (!_package_list.empty()) {
        MediaTaskPackage pkg = _package_list.front();
        _package_list.pop_front();

        for (auto item : pkg._media_list) {
            _readyPlay.push_back(item);
        }
        cache_ = pkg;
        _time_counter = 0;
    }
}

void RobotMedia::UpdateCache()
{
    if (_time_counter >= _time_threshold) {
        _time_counter = 0;
        if (cache_ != default_media_task_pkg) {
            cache_ = MediaTaskPackage();
        }
    }
    _time_counter += 800;
};

void RobotMedia::Init() { thread_ = std::thread(&RobotMedia::Loop, this); }

void RobotMedia::Loop()
{
    std::shared_ptr<MediaPlay> media;
    bool isPlay = true;
    while (1) {
        {
            std::unique_lock<std::mutex> lock(_mutex);

            if ((isPlay == true) && (_readyPlay.size() > 0)) {
                media = _readyPlay.front();
                _readyPlay.pop_front();
                isPlay = false;
            }
        }
        if (isPlay == false) {
            media->Play();
            isPlay = true;
        }

        if (_readyPlay.empty()) {
            Unpack();
            TimerTools::SleepForMs(800);
            UpdateCache();
        }
    }
}

void AddMediaPackage(const MediaTaskPackage &pkg) { RobotMedia::GetInstance().AddPackage(pkg); }

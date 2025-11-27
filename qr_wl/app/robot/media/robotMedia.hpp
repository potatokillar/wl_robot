
#pragma once

#include <deque>
#include <map>
#include <memory>
#include <mutex>

#include "baseline.hpp"
#include "wavPlay.hpp"

class MediaTaskPackage
{
public:
    MediaTaskPackage() = default;
    MediaTaskPackage(const std::string &name);

    void Add(const std::string &name);
    bool operator==(const MediaTaskPackage &other) const;  // 重载 == 运算符
    bool operator!=(const MediaTaskPackage &other) const;  // 重载 != 运算符

    std::vector<std::shared_ptr<MediaPlay>> _media_list;

private:
};

class RobotMedia : public Singleton<RobotMedia>  // , public MediaCache
{
public:
    RobotMedia() { Init(); };
    virtual ~RobotMedia();

    void AddPlay(std::shared_ptr<MediaPlay> mediaPlay);
    void AddPackage(const MediaTaskPackage &pkg);
    void Unpack();
    void UpdateCache();

    MediaTaskPackage cache_;

private:
    void Init() override;
    void Loop() override;
    std::deque<std::shared_ptr<MediaPlay>> _readyPlay;
    std::deque<MediaTaskPackage> _package_list;
    std::mutex _mutex;
    uint32_t _time_threshold = {1500};
    uint32_t _time_counter = {0};
};

void AddMediaPackage(const MediaTaskPackage &pkg);
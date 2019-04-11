#pragma once

#include <vector>
#include <opencv2/core.hpp>
#include <arv.h>
#include "VideoSystem.h"

class VideoSystemImpl : public VideoSystem
{
public:

    VideoSystemImpl();
    ~VideoSystemImpl();

    bool initialize() override;
    void finalize() override;

    int getNumberOfGenICamCameras() override;
    std::string getNameOfGenICamCamera(int idx) override;

    GenICamVideoSourcePtr createGenICamVideoSource(const std::vector<int>& camera_ids) override;

    VideoSourcePtr createVideoSourceMockMono() override;
    VideoSourcePtr createVideoSourceMockStereo() override;

private:

    std::vector<std::string> mGenICamCameras;
};


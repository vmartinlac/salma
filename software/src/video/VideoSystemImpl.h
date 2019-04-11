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

    GenICamVideoSourcePtr createGenICamVideoSourceMono(int camera_idx) override;
    GenICamVideoSourcePtr createGenICamVideoSourceStereo(int left_camera_idx, int right_camera_idx) override;

    VideoSourcePtr createVideoSourceMockMono() override;
    VideoSourcePtr createVideoSourceMockStereo() override;

private:

    std::vector<std::string> mGenICamCameras;
};


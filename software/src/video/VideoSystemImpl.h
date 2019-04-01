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

    VideoSourcePtr createVideoSourceGenICamMono(int camera_idx, bool software_trigger) override;
    VideoSourcePtr createVideoSourceGenICamStereo(int left_camera_idx, int right_camera_id, bool software_trigger) override;

    VideoSourcePtr createVideoSourceMockMono() override;
    VideoSourcePtr createVideoSourceMockStereo() override;

private:

    std::vector<std::string> mGenICamCameras;
};


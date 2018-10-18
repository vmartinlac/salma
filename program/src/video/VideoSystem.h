#pragma once

#include <vector>
#include <memory>
#include <mutex>
#include <atomic>
#include <opencv2/core.hpp>
#include "VideoSource.h"
#include "AvtCamera.h"

class VideoSystem
{
public:

    static bool initialize();
    static void finalize();
    static VideoSystem* instance();

public:

    VideoSystem();
    ~VideoSystem();

    VideoSourcePtr createMonoAvtVideoSource(int camera_idx);
    VideoSourcePtr createStereoAvtVideoSource(int left_camera_idx, int right_camera_id);
    VideoSourcePtr createOpenCVVideoSource(const std::string& filename);
    VideoSourcePtr createOpenCVVideoSource(int id);
    VideoSourcePtr createVideoSourceFromMonoRecording(const std::string& path);
    VideoSourcePtr createVideoSourceFromStereoRecording(const std::string& path);
    VideoSourcePtr createMockMonoVideoSource();
    VideoSourcePtr createMockStereoVideoSource();
    VideoSourcePtr assembleVideoSources(const std::vector<VideoSourcePtr>& video_sources);

    bool detectAvtCameras();
    int getNumberOfAvtCameras();
    std::string getNameOfAvtCamera(int idx);

private:

    static std::unique_ptr<VideoSystem> mInstance;
    std::vector<AvtCameraPtr> mAvtCameras;

protected:

    bool doInitialize();
    void doFinalize();
    void clearAvtCameras();
};


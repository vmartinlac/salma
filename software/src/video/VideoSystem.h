#pragma once

#include <string>
#include "VideoSource.h"
#include "GenICamVideoSource.h"

class VideoSystem
{
public:

    static VideoSystem* instance();

    virtual bool initialize() = 0;
    virtual void finalize() = 0;

public:

    virtual ~VideoSystem();

    virtual int getNumberOfGenICamCameras();
    virtual std::string getNameOfGenICamCamera(int idx);

    virtual GenICamVideoSourcePtr createGenICamVideoSource(const std::vector<int>& camera_ids);

    virtual VideoSourcePtr createOpenCVVideoSource(int id);
    virtual VideoSourcePtr createOpenCVVideoSource(const std::string& filename);

    virtual VideoSourcePtr createVideoSourceMockMono();
    virtual VideoSourcePtr createVideoSourceMockStereo();

protected:

    VideoSystem();

private:

    static std::unique_ptr<VideoSystem> mInstance;
};


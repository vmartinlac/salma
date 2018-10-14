#include "VideoSystem.h"

class OpenCVFileVideoSource : public VideoSource
{
};

class OpenCVCameraVideoSource : public VideoSource
{
};

class AvtMonoVideoSource : public VideoSource
{
};

class AvtStereoVideoSource : public VideoSource
{
};

class MonoRecordingVideoSource : public VideoSource
{
};

class StereoRecordingVideoSource : public VideoSource
{
};

VideoSystem::VideoSystem()
{
    if(mInstance != nullptr)
    {
        // some problem!
        exit(1);
    }

    mInstance = this;
}

VideoSystem::~VideoSystem()
{
    if(mInstance == nullptr)
    {
        // some problem!
        exit(1);
    }
    mInstance = nullptr;
}

VideoSystem* VideoSystem::mInstance;

bool VideoSystem::initialize()
{
    return true;
}

void VideoSystem::finalize()
{
}

VideoSystem& VideoSystem::instance()
{
    return *mInstance;
}


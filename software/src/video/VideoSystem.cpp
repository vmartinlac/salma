#include "VideoSystem.h"
#include "VideoSystemImpl.h"

std::unique_ptr<VideoSystem> VideoSystem::mInstance;

VideoSystem* VideoSystem::instance()
{
    if( bool(mInstance) == false)
    {
        mInstance.reset(new VideoSystemImpl());
    }

    return mInstance.get();
}

VideoSystem::VideoSystem()
{
}

VideoSystem::~VideoSystem()
{
}

int VideoSystem::getNumberOfGenICamCameras()
{
    return 0;
}

std::string VideoSystem::getNameOfGenICamCamera(int idx)
{
    throw std::runtime_error("fatal error");
}

GenICamVideoSourcePtr VideoSystem::createGenICamVideoSource(const std::vector<int>& camera_ids)
{
    return GenICamVideoSourcePtr();
}

VideoSourcePtr VideoSystem::createOpenCVVideoSource(int id)
{
    return VideoSourcePtr();
}

VideoSourcePtr VideoSystem::createOpenCVVideoSource(const std::string& filename)
{
    return VideoSourcePtr();
}

VideoSourcePtr VideoSystem::createVideoSourceMockMono()
{
    return VideoSourcePtr();
}

VideoSourcePtr VideoSystem::createVideoSourceMockStereo()
{
    return VideoSourcePtr();
}


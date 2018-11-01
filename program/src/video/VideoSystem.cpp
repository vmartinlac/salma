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

VideoSourcePtr VideoSystem::createVideoSourceGenICamMono(int camera_idx)
{
    return VideoSourcePtr();
}

VideoSourcePtr VideoSystem::createVideoSourceGenICamStereo(int left_camera_idx, int right_camera_id)
{
    return VideoSourcePtr();
}

VideoSourcePtr VideoSystem::createVideoSourceOpenCV(int id)
{
    return VideoSourcePtr();
}

VideoSourcePtr VideoSystem::createVideoSourceOpenCV(const std::string& filename)
{
    return VideoSourcePtr();
}

VideoSourcePtr VideoSystem::createVideoSourceFromFileMono(const std::string& path)
{
    return VideoSourcePtr();
}

VideoSourcePtr VideoSystem::createVideoSourceFromFileStereo(const std::string& path)
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


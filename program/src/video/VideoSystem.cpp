#include "VideoSystem.h"

VideoSystem::VideoSystem()
{
}

VideoSystem::~VideoSystem()
{
}

std::unique_ptr<VideoSystem> VideoSystem::mInstance;

bool VideoSystem::initialize()
{
    mInstance.reset(new VideoSystem);

    return true;
}

void VideoSystem::finalize()
{
    mInstance.reset();
}

VideoSystem& VideoSystem::instance()
{
    return *mInstance;
}


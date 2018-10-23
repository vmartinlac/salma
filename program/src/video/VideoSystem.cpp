#include "GeneralVideoSource.h"
#include "VideoSystem.h"

//#define TEST_WITHOUT_VIMBA

std::unique_ptr<VideoSystem> VideoSystem::mInstance;

bool VideoSystem::initialize()
{
    mInstance.reset(new VideoSystem());

    const bool ret = mInstance->doInitialize();

    if(ret == false)
    {
        mInstance.reset();
    }

    return ret;
}

void VideoSystem::finalize()
{
    mInstance->doFinalize();
    mInstance.reset();
}

VideoSystem* VideoSystem::instance()
{
    return mInstance.get();
}

VideoSystem::VideoSystem()
{
}

VideoSystem::~VideoSystem()
{
}

bool VideoSystem::doInitialize()
{
    bool ok = true;

#ifndef TEST_WITHOUT_VIMBA
    if(ok)
    {
        ok = (VmbErrorSuccess == VmbStartup());
    }

    if(ok)
    {
        bool gige;
        ok = ( VmbFeatureBoolGet(gVimbaHandle, "GeVTLIsPresent", &gige) == VmbErrorSuccess && gige );
    }

    if(ok)
    {
        detectAvtCameras();
    }
#endif

    return ok;
}

void VideoSystem::doFinalize()
{
    clearAvtCameras();
#ifndef TEST_WITHOUT_VIMBA
    VmbShutdown();
#endif
}

void VideoSystem::clearAvtCameras()
{
    for( AvtCameraPtr& camera : mAvtCameras)
    {
        if(camera.use_count() != 1)
        {
            throw std::runtime_error("Attempted to release VimbaCameraManager while a VimbaCamera is still referenced.");
        }
    }

    mAvtCameras.clear();
}

bool VideoSystem::detectAvtCameras()
{
    bool ok = true;
    VmbUint32_t count = 0;
    std::vector<VmbCameraInfo_t> info;

    clearAvtCameras();

#ifndef TEST_WITHOUT_VIMBA

    // discover.

    if(ok)
    {
        ok = (VmbErrorSuccess == VmbFeatureCommandRun(gVimbaHandle, "GeVDiscoveryAllOnce"));
    }

    // get number of cameras.

    if(ok)
    {
        ok = (VmbErrorSuccess == VmbCamerasList(NULL, 0, &count, sizeof(VmbCameraInfo_t)));
    }

    // retrieve camera infos.

    if( ok && count > 0 )
    {
        info.resize(count);
        ok = (VmbErrorSuccess == VmbCamerasList(&info.front(), count, &count, sizeof(VmbCameraInfo_t)));
    }

    // create cameras.

    if(ok && count > 0 )
    {
        mAvtCameras.resize(count);

        for(int i=0; i<count; i++)
        {
            mAvtCameras[i].reset( new AvtCamera(info[i]) );
        }
    }
#endif

    if(ok == false)
    {
        mAvtCameras.clear();
    }

    return ok;
}

VideoSourcePtr VideoSystem::createMonoAvtVideoSource(int camera_idx)
{
    GeneralVideoSourcePtr ret;

    if( 0 <= camera_idx && camera_idx < mAvtCameras.size() )
    {
        ret.reset(new GeneralVideoSource());
        ret->set( mAvtCameras[camera_idx], TriggerPtr() );
    }

    return ret;
}

VideoSourcePtr VideoSystem::createStereoAvtVideoSource(int left_camera_idx, int right_camera_idx)
{
    GeneralVideoSourcePtr ret;

    if( left_camera_idx != right_camera_idx && 0 <= left_camera_idx && left_camera_idx < mAvtCameras.size() && 0 <= right_camera_idx && right_camera_idx < mAvtCameras.size() )
    {
        ret.reset(new GeneralVideoSource());
        ret->set( mAvtCameras[left_camera_idx], mAvtCameras[right_camera_idx], TriggerPtr() );
    }

    return ret;
}

int VideoSystem::getNumberOfAvtCameras()
{
    return mAvtCameras.size();
}

std::string VideoSystem::getNameOfAvtCamera(int idx)
{
    return mAvtCameras[idx]->getHumanName();
}


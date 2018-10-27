#include <iostream>
#include "AvtRig.h"
#include "GeneralVideoSource.h"
#include "VideoSystem.h"

std::unique_ptr<VideoSystem> VideoSystem::mInstance;

VideoSystem* VideoSystem::instance()
{
    if(bool(mInstance) == false)
    {
        mInstance.reset(new VideoSystem());
    }

    return mInstance.get();
}

VideoSystem::VideoSystem()
{
}

VideoSystem::~VideoSystem()
{
}

bool VideoSystem::initialize()
{
    bool ok = true;

    AVT::VmbAPI::VimbaSystem& vimba = AVT::VmbAPI::VimbaSystem::GetInstance();

    if(ok)
    {
        ok = (VmbErrorSuccess == vimba.Startup());

        if(ok == false)
        {
            std::cerr << "Could not initialize Vimba!" << std::endl;
        }
    }

    /*
    if(ok)
    {
        bool gige;
        ok = ( VmbFeatureBoolGet(gVimbaHandle, "GeVTLIsPresent", &gige) == VmbErrorSuccess && gige );
    }
    */

    if(ok)
    {
        mAvtCameras.clear();

        ok = (VmbErrorSuccess == vimba.GetCameras(mAvtCameras));

        if(ok == false)
        {
            std::cerr << "Error while enumerating cameras!" << std::endl;
        }
    }

    return ok;
}

void VideoSystem::finalize()
{
    AVT::VmbAPI::VimbaSystem::GetInstance().Shutdown();
}

/*
void VideoSystem::clearAvtCameras()
{
    for( AVT::VmbAPI::CameraPtr& camera : mAvtCameras)
    {
        if(camera.use_count() != 1)
        {
            //throw std::runtime_error("Attempted to release VimbaCameraManager while a VimbaCamera is still referenced.");
            std::cerr << "Releasing VideoSystem while some camera is still referenced!" << std::endl;
        }
    }

    mAvtCameras.clear();
}
*/

VideoSourcePtr VideoSystem::createMonoAvtVideoSource(int camera_idx)
{
    AvtCameraPtr ret;

    if( 0 <= camera_idx && camera_idx < mAvtCameras.size() )
    {
        ret.reset(new AvtCamera(mAvtCameras[camera_idx]));
    }

    return ret;
}

VideoSourcePtr VideoSystem::createStereoAvtVideoSource(int left_camera_idx, int right_camera_idx)
{
    AvtRigPtr ret;

    bool ok = true;

    ok = ok && ( left_camera_idx != right_camera_idx );
    ok = ok && ( 0 <= left_camera_idx && left_camera_idx < mAvtCameras.size() );
    ok = ok && ( 0 <= right_camera_idx && right_camera_idx < mAvtCameras.size() );

    if(ok)
    {
        auto left = mAvtCameras[left_camera_idx];
        auto right = mAvtCameras[right_camera_idx];
        ret.reset(new AvtRig({left, right}));
    }

    return ret;
}

int VideoSystem::getNumberOfAvtCameras()
{
    return mAvtCameras.size();
}

std::string VideoSystem::getNameOfAvtCamera(int idx)
{
    AvtCameraPtr cam(new AvtCamera(mAvtCameras[idx]));
    return cam->getHumanName();
}


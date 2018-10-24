#include <iostream>
#include "GeneralVideoSource.h"
#include "VideoSystem.h"

//#define TEST_WITHOUT_VIMBA

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
        ok = detectAvtCameras();

        if(ok == false)
        {
            std::cerr << "Error while enumerating cameras!" << std::endl;
        }
    }

    return ok;
}

void VideoSystem::finalize()
{
    clearAvtCameras();

    AVT::VmbAPI::VimbaSystem::GetInstance().Shutdown();
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
    AVT::VmbAPI::VimbaSystem& vimba = AVT::VmbAPI::VimbaSystem::GetInstance();

    AVT::VmbAPI::CameraPtrVector cameras;

    bool ok = true;

    //VmbUint32_t count = 0;

    clearAvtCameras();

    // discover.

    if(ok)
    {
        ok = (VmbErrorSuccess == vimba.GetCameras(cameras));
    }

    // create cameras.

    if(ok)
    {
        mAvtCameras.resize(cameras.size());

        for(int i=0; i<cameras.size(); i++)
        {
            mAvtCameras[i].reset( new AvtCamera(cameras[i]) );
        }
    }

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


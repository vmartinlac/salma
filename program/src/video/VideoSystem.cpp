#include <iostream>
#include "VideoReader.h"
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
    mHaveVimba = false;
}

VideoSystem::~VideoSystem()
{
}

bool VideoSystem::initialize(bool with_vimba)
{
    mHaveVimba = with_vimba;

    bool ok = true;

    if(with_vimba)
    {
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
    }

    return ok;
}

void VideoSystem::finalize()
{
    if(mHaveVimba)
    {
        AVT::VmbAPI::VimbaSystem::GetInstance().Shutdown();
    }
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
    VideoSourcePtr ret;

    if( mHaveVimba && 0 <= camera_idx && camera_idx < mAvtCameras.size() )
    {
        //ret.reset(new AvtCamera(mAvtCameras[camera_idx]));
        ret.reset(new AvtRig({mAvtCameras[camera_idx]}));
    }

    return ret;
}

VideoSourcePtr VideoSystem::createStereoAvtVideoSource(int left_camera_idx, int right_camera_idx)
{
    AvtRigPtr ret;

    bool ok = true;

    ok = ok && mHaveVimba;
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
    if(mHaveVimba)
    {
        return mAvtCameras.size();
    }
    else
    {
        return 0;
    }
}

std::string VideoSystem::getNameOfAvtCamera(int idx)
{
    if(mHaveVimba)
    {
        AvtCameraPtr cam(new AvtCamera(mAvtCameras[idx]));
        return cam->getHumanName();
    }
    else
    {
        throw std::runtime_error("Vimba was not initialized!");
    }
}

VideoSourcePtr VideoSystem::createVideoSourceFromMonoRecording(const std::string& path)
{
    VideoReaderPtr ret(new VideoReader(1));

    ret->setPath(path);

    return ret;
}

VideoSourcePtr VideoSystem::createVideoSourceFromStereoRecording(const std::string& path)
{
    VideoReaderPtr ret(new VideoReader(2));

    ret->setPath(path);

    return ret;
}


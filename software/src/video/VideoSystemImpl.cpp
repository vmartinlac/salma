#define QT_NO_KEYWORDS

#include <iostream>
#include "ArduinoTrigger.h"
#include "MockCamera.h"
#include "GenICamRig.h"
#include "VideoSystemImpl.h"

/*
#define MOCK_GENICAM_CAMERAS
#define MOCK_GENICAM_CAMERA_WIDTH 1292
#define MOCK_GENICAM_CAMERA_HEIGHT 964
*/

VideoSystemImpl::VideoSystemImpl()
{
}

VideoSystemImpl::~VideoSystemImpl()
{
}

bool VideoSystemImpl::initialize()
{
    //arv_g_type_init();
    arv_update_device_list();

    unsigned int N = arv_get_n_devices();

    mGenICamCameras.clear();
    mGenICamCameras.resize(N);
    for(unsigned int i=0; i<N; i++)
    {
        mGenICamCameras[i] = std::string(arv_get_device_id(i));
    }

    return true;
}

void VideoSystemImpl::finalize()
{
    mGenICamCameras.clear();
    arv_shutdown();
}

int VideoSystemImpl::getNumberOfGenICamCameras()
{
#ifdef MOCK_GENICAM_CAMERAS
    return 2;
#else
    return mGenICamCameras.size();
#endif
}

std::string VideoSystemImpl::getNameOfGenICamCamera(int idx)
{
#ifdef MOCK_GENICAM_CAMERAS
    const char* ret = nullptr;
    switch(idx)
    {
    case 0:
        ret = "Mock GenICam camera #0";
        break;
    case 1:
        ret = "Mock GenICam camera #1";
        break;
    default:
        throw std::runtime_error("internal error");
    }
    return ret;
#else
    return mGenICamCameras.at(idx);
#endif
}

GenICamVideoSourcePtr VideoSystemImpl::createGenICamVideoSource(const std::vector<int>& camera_ids)
{
#ifdef MOCK_GENICAM_CAMERAS

    return MockCameraPtr(new MockCamera(camera_ids.size(), MOCK_GENICAM_CAMERA_WIDTH, MOCK_GENICAM_CAMERA_HEIGHT));

#else

    std::vector<std::string> ids(camera_ids.size());
    GenICamRigPtr ret;

    bool ok = true;

    if(ok)
    {
        ok = (camera_ids.empty() == false);
    }

    for(size_t i=0; ok && i<camera_ids.size(); i++)
    {
        ok = ( 0 <= camera_ids[i] && camera_ids[i] < mGenICamCameras.size() );

        if(ok)
        {
            ids[i] = mGenICamCameras[camera_ids[i]];
        }
    }

    if(ok)
    {
        ret.reset( new GenICamRig( ids ) );
    }

    return ret;

#endif
}

VideoSourcePtr VideoSystemImpl::createVideoSourceMockMono()
{
    return MockCameraPtr(new MockCamera(1, 640, 480));
}

VideoSourcePtr VideoSystemImpl::createVideoSourceMockStereo()
{
    return MockCameraPtr(new MockCamera(2, 640, 480));
}


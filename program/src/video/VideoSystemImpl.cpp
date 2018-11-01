#define QT_NO_KEYWORDS

#include <iostream>
#include "VideoReader.h"
#include "MockCamera.h"
#include "GenICamRig.h"
#include "VideoSystemImpl.h"

VideoSystemImpl::VideoSystemImpl()
{
}

VideoSystemImpl::~VideoSystemImpl()
{
}

bool VideoSystemImpl::initialize()
{
    arv_g_type_init();
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
    return mGenICamCameras.size();
}

std::string VideoSystemImpl::getNameOfGenICamCamera(int idx)
{
    return mGenICamCameras.at(idx);
}

VideoSourcePtr VideoSystemImpl::createVideoSourceGenICamMono(int camera_idx)
{
    GenICamRigPtr ret;

    const bool ok = (0 <= camera_idx && camera_idx < mGenICamCameras.size());

    if(ok)
    {
        std::string& id = mGenICamCameras.at(camera_idx);
        ret.reset(new GenICamRig{ id });
    }

    return ret;
}

VideoSourcePtr VideoSystemImpl::createVideoSourceGenICamStereo(int left_camera_idx, int right_camera_idx)
{
    GenICamRigPtr ret;
    bool ok = true;

    ok = ok && (left_camera_idx != right_camera_idx);
    ok = ok && (0 <= left_camera_idx && left_camera_idx < mGenICamCameras.size());
    ok = ok && (0 <= right_camera_idx && right_camera_idx < mGenICamCameras.size());

    if(ok)
    {
        std::string& left_id = mGenICamCameras[left_camera_idx];
        std::string& right_id = mGenICamCameras[right_camera_idx];

        ret.reset(new GenICamRig{left_id, right_id});
    }

    return ret;
}

VideoSourcePtr VideoSystemImpl::createVideoSourceFromFileMono(const std::string& path)
{
    VideoReaderPtr ret(new VideoReader(1));
    ret->setPath(path);
    return ret;
}

VideoSourcePtr VideoSystemImpl::createVideoSourceFromFileStereo(const std::string& path)
{
    VideoReaderPtr ret(new VideoReader(2));
    ret->setPath(path);
    return ret;
}

VideoSourcePtr VideoSystemImpl::createVideoSourceMockMono()
{
    return MockCameraPtr(new MockCamera(1, 640, 480));
}

VideoSourcePtr VideoSystemImpl::createVideoSourceMockStereo()
{
    return MockCameraPtr(new MockCamera(2, 640, 480));
}


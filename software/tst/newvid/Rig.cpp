#include "Rig.h"

void Rig::RigProc(Rig* rig)
{
}

Rig::Rig(const std::initializer_list<int>& cams)
{
    for(int i : cams)
    {
        const std::string id = arv_get_device_id(i);
        mCameras.emplace_back( new Camera(this, id, i) );
    }
}

void Rig::open()
{
    mThread = std::thread(RigProc, this);

    for( CameraPtr cam : mCameras )
    {
        cam->open();
    }
}

void Rig::close()
{
    mThread.join();
    for( CameraPtr cam : mCameras )
    {
        cam->close();
    }
}

cv::Mat Rig::read()
{
    return cv::Mat();
}


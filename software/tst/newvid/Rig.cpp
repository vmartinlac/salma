#include "Rig.h"

void Rig::RigProc(Rig* rig)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
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
    for( CameraPtr cam : mCameras )
    {
        cam->open();
    }

    mIsOpen = true;

    mThread = std::thread(RigProc, this);
}

void Rig::close()
{
    mIsOpen = false;

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


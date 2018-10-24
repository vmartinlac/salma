#include <thread>
#include <algorithm>
#include <chrono>
#include "GeneralVideoSource.h"

GeneralVideoSource::GeneralVideoSource()
{
}

GeneralVideoSource::~GeneralVideoSource()
{
}

void GeneralVideoSource::set(CameraPtr camera, TriggerPtr trigger)
{
    mCameras.resize(1);
    mCameras[0] = camera;
    mTrigger = trigger;
}

void GeneralVideoSource::set(CameraPtr camera0, CameraPtr camera1, TriggerPtr trigger)
{
    mCameras.resize(2);
    mCameras[0] = camera0;
    mCameras[1] = camera1;
    mTrigger = trigger;
}

void GeneralVideoSource::set(CameraPtr camera0, CameraPtr camera1, CameraPtr camera2, TriggerPtr trigger)
{
    mCameras.resize(3);
    mCameras[0] = camera0;
    mCameras[1] = camera1;
    mCameras[2] = camera2;
    mTrigger = trigger;
}

std::string GeneralVideoSource::getHumanName()
{
    std::string ret;

    ret += "{ ";

    bool first = true;

    for(CameraPtr cam : mCameras)
    {
        if(first)
        {
            first = false;
        }
        else
        {
            ret += " + ";
        }

        ret += cam->getHumanName();
    }

    ret += " }";

    return ret;
}

bool GeneralVideoSource::open()
{
    bool ok = true;

    for(CameraPtr& cam : mCameras)
    {
        ok = ok && cam->open();
    }

    if(mTrigger)
    {
        ok = ok && mTrigger->open();
    }

    if(ok == false)
    {
        close();
    }

    return ok;
}

void GeneralVideoSource::close()
{
    if(mTrigger)
    {
        mTrigger->close();
    }

    for(CameraPtr& cam : mCameras)
    {
        cam->close();
    }
}

void GeneralVideoSource::trigger()
{
    if(mTrigger)
    {
        mTrigger->trigger();
    }
    else
    {
        for(CameraPtr& cam : mCameras)
        {
            cam->trigger();
        }
    }
}

void GeneralVideoSource::read(Image& image)
{
    const int N = mCameras.size();

    std::vector<bool> received(N, false);
    std::vector<cv::Mat> frames(N);
    double timestamp = 0.0;
    int max_rounds = 6;

    bool go_on = true;

    while(go_on && max_rounds > 0)
    {
        max_rounds--;
        go_on = false;

        for(int i=0; i<N; i++)
        {
            if(received[i] == false)
            {
                Image im;
                mCameras[i]->read(im);

                if( im.isValid() )
                {
                    if( im.getNumberOfFrames() != 1 ) throw std::runtime_error("internal error");

                    if(i == 0)
                    {
                        timestamp = im.getTimestamp();
                    }

                    frames[i] = im.getFrame(0);
                    received[i] = true;
                }
                else
                {
                    go_on = true;
                }
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if( std::all_of(received.begin(), received.end(), [] (bool a) { return a; }) )
    {
        image.setValid(timestamp, frames);
    }
    else
    {
        image.setInvalid();
    }
}

int GeneralVideoSource::getNumberOfCameras()
{
    return mCameras.size();
}


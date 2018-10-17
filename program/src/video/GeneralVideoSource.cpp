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
}

void GeneralVideoSource::read(Image& image)
{
    double timestamp = 0.0;

    std::vector<cv::Mat> frames;
    frames.resize(mCameras.size());

    bool ok = true;

    for(int i=0; ok && i<mCameras.size(); i++)
    {
        Image tmp;
        mCameras[i]->read(tmp);

        if( tmp.isValid() )
        {
            if( tmp.getNumberOfFrames() != 1 ) throw std::runtime_error("internal error");

            if(i == 0)
            {
                timestamp = tmp.getTimestamp();
            }

            frames[i] = tmp.getFrame(0);
        }
        else
        {
            ok = false;
        }
    }

    if(ok)
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


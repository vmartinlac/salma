#include <iostream>
#include "AvtStereoCamera.h"

class AvtStereoCamera::FrameObserver : public AVT::VmbAPI::IFrameObserver
{
public:
    FrameObserver(AvtStereoCamera* cam, int idx) : IFrameObserver( cam->mCameras[idx].camera )
    {
        mCamera = cam;
        mIndex = idx;
    }

    void FrameReceived(const AVT::VmbAPI::FramePtr frame) override
    {
        // TODO!
        //mCamera->mCameras[mIndex].image.set_value(std::move(image));
        m_pCamera->QueueFrame(frame);
    }

protected:

    AvtStereoCamera* mCamera;
    int mIndex;
};

AvtStereoCamera::AvtStereoCamera()
{
}

AvtStereoCamera::~AvtStereoCamera()
{
}

void AvtStereoCamera::setCamera(AVT::VmbAPI::CameraPtr camera)
{
    mCameras.resize(1);
    mCameras[0].camera = camera;
}

void AvtStereoCamera::setCameras(AVT::VmbAPI::CameraPtr left_camera, AVT::VmbAPI::CameraPtr right_camera)
{
    mCameras.resize(2);
    mCameras[0].camera = left_camera;
    mCameras[1].camera = right_camera;
}

std::string AvtStereoCamera::getHumanName()
{
    std::string ret;

    ret += "{ ";

    for(size_t i=0; i<mCameras.size(); i++)
    {
        if(i > 0)
        {
            ret += " + ";
        }

        std::string camera_id;
        std::string interface_id;

        bool ok = true;
        ok = ok && (VmbErrorSuccess == mCameras[i].camera->GetID(camera_id));
        ok = ok && (VmbErrorSuccess == mCameras[i].camera->GetInterfaceID(interface_id));

        if(ok)
        {
            ret += interface_id + ":" + camera_id;
        }
        else
        {
            ret += "UNNAMED_CAMERA";
        }
    }

    ret += " }";

    return ret;
}

bool AvtStereoCamera::open()
{
    bool ok = true;

    for(size_t i=0; ok && i<mCameras.size(); i++)
    {
        AVT::VmbAPI::CameraPtr cam = mCameras[i].camera;

        AVT::VmbAPI::FeaturePtr feature;

        ok = ok && ( VmbErrorSuccess == cam->Open(VmbAccessModeFull) );

        ok = ok && ( VmbErrorSuccess == cam->GetFeatureByName("TriggerSource", feature) ) && ( VmbErrorSuccess == feature->SetValue("Software") );

        ok = ok && ( VmbErrorSuccess == cam->GetFeatureByName("PixelFormat", feature) ) && ( VmbErrorSuccess == feature->SetValue("BGR8Packed") );

        ok = ok && ( VmbErrorSuccess == cam->GetFeatureByName("PayloadSize", feature) ) && ( VmbErrorSuccess == feature->GetValue(mCameras[i].payload_size) );

        ok = ok && ( VmbErrorSuccess == cam->GetFeatureByName("GevTimestampTickFrequency", feature) ) && ( VmbErrorSuccess == feature->GetValue(mCameras[i].tick_frequency) );

        if(ok)
        {
            mCameras[i].frame.reset(new AVT::VmbAPI::Frame(mCameras[i].payload_size));

            mCameras[i].frame->RegisterObserver( AVT::VmbAPI::IFrameObserverPtr(new FrameObserver(this, i)) );
        }

        ok = ok && ( VmbErrorSuccess == cam->AnnounceFrame(mCameras[i].frame) );

        ok = ok && ( VmbErrorSuccess == cam->StartCapture() );

        ok = ok && ( VmbErrorSuccess == cam->QueueFrame(mCameras[i].frame) );

        ok = ok && ( VmbErrorSuccess == cam->GetFeatureByName("AcquisitionStart", feature) ) && ( VmbErrorSuccess == feature->RunCommand() );
    }

    return ok;
}

void AvtStereoCamera::close()
{
    bool ok = true;

    for(size_t i=0; ok && i<mCameras.size(); i++)
    {
        AVT::VmbAPI::FeaturePtr feature;

        AVT::VmbAPI::CameraPtr cam = mCameras[i].camera;

        ok = ok && ( VmbErrorSuccess == cam->GetFeatureByName("AcquisitionStop", feature) ) && ( VmbErrorSuccess == feature->RunCommand() );

        ok = ok && ( VmbErrorSuccess == cam->EndCapture() );

        ok = ok && ( VmbErrorSuccess == cam->FlushQueue() );

        ok = ok && ( VmbErrorSuccess == cam->RevokeAllFrames() );

        ok = ok && ( VmbErrorSuccess == cam->Close() );

        mCameras[i].frame.reset();
    }

    if( ok == false )
    {
        std::cerr << "Error while closing the camera!" << std::endl;
    }
}

void AvtStereoCamera::trigger()
{
    for(size_t i=0; i<mCameras.size(); i++)
    {
        mCameras[i].camera->FlushQueue();
        mCameras[i].camera->QueueFrame( mCameras[i].frame );
    } 

    bool ok = true;

    for(size_t i=0; i<mCameras.size(); i++)
    {
        AVT::VmbAPI::FeaturePtr feature;

        ok = ok && ( mCameras[i].camera->GetFeatureByName("TriggerSoftware", feature) == VmbErrorSuccess );
        ok = ok && ( feature->RunCommand() == VmbErrorSuccess );
    }

    if(ok == false)
    {
        std::cerr << "Error while triggering the camera!" << std::endl;
    }
}

void AvtStereoCamera::read(Image& image)
{
    bool ok = true;

    std::chrono::time_point< std::chrono::steady_clock > until = std::chrono::steady_clock::now() + std::chrono::milliseconds(1000);

    for(size_t i=0; ok && i<mCameras.size(); i++)
    {
        std::future_status status = mCameras[i].image.get_future().wait_until(until);

        ok = ok && ( status == std::future_status::ready );
    }

    std::vector<Image> images(mCameras.size());

    if(ok)
    {
        for(size_t i=0; ok && i<mCameras.size(); i++)
        {
            images[i] = mCameras[i].image.get_future().get();
            ok = ok && images[i].isValid();
        }
    }

    if(ok)
    {
        double timestamp = images.front().getTimestamp();

        std::vector<cv::Mat> frames(mCameras.size());

        for(size_t i=0; i<mCameras.size(); i++)
        {
            frames[i] = images[i].getFrame();
        }

        image.setValid(timestamp, frames);
    }
    else
    {
        image.setInvalid();
    }
}

int AvtStereoCamera::getNumberOfCameras()
{
    return mCameras.size();
}


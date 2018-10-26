#include <iostream>
#include "AvtRig.h"

class AvtRig::CameraData
{
public:

    AVT::VmbAPI::CameraPtr avt_camera;
    AVT::VmbAPI::FramePtrVector frames;

    std::mutex mutex;
    std::condition_variable condition;
    Image new_image;

    VmbInt64_t tick_frequency;
    VmbInt64_t payload_size;
};

class AvtRig::FrameObserver : public AVT::VmbAPI::IFrameObserver
{
public:

    FrameObserver(CameraDataPtr cam) : IFrameObserver( cam->avt_camera )
    {
        mCamera = cam;
    }

    void FrameReceived(const AVT::VmbAPI::FramePtr frame) override
    {
        VmbUchar_t* buffer;
        VmbUint64_t timestamp;
        VmbUint32_t width;
        VmbUint32_t height;
        VmbFrameStatusType status;

        bool ok = true;
        
        ok = ok && (frame->GetImage(buffer) == VmbErrorSuccess);
        ok = ok && (frame->GetReceiveStatus(status) == VmbErrorSuccess);
        ok = ok && (frame->GetTimestamp(timestamp) == VmbErrorSuccess);
        ok = ok && (frame->GetWidth(width) == VmbErrorSuccess);
        ok = ok && (frame->GetHeight(height) == VmbErrorSuccess);

        if(ok && status == VmbFrameStatusComplete && width > 0 && height > 0)
        {
            const double t = double(timestamp) / double(mCamera->tick_frequency);

            cv::Mat wrapper(
                cv::Size(width, height),
                CV_8UC3,
                buffer);

            Image image;
            image.setValid(t, wrapper.clone());

            {
                std::lock_guard<std::mutex> lock(mCamera->mutex);
                mCamera->new_image = std::move(image);
            }

            mCamera->condition.notify_one();
        }
        else
        {
            std::cerr << "Incorrect frame received!" << std::endl;
        }

        m_pCamera->QueueFrame(frame);
    }

protected:

    CameraDataPtr mCamera;
};

AvtRig::AvtRig(AVT::VmbAPI::CameraPtr camera)
{
    mCameras.resize(1);

    mCameras[0].reset(new CameraData());
    mCameras[0]->avt_camera = camera;
}

AvtRig::AvtRig(AVT::VmbAPI::CameraPtr left_camera, AVT::VmbAPI::CameraPtr right_camera)
{
    mCameras.resize(2);

    mCameras[0].reset(new CameraData());
    mCameras[0]->avt_camera = left_camera;

    mCameras[1].reset(new CameraData());
    mCameras[1]->avt_camera = right_camera;
}

AvtRig::~AvtRig()
{
}

std::string AvtRig::getHumanName()
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
        ok = ok && (VmbErrorSuccess == mCameras[i]->avt_camera->GetID(camera_id));
        ok = ok && (VmbErrorSuccess == mCameras[i]->avt_camera->GetInterfaceID(interface_id));

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

bool AvtRig::open()
{
    bool ok = true;

    std::cerr << "Opening the camera." << std::endl;

    for(size_t i=0; ok && i<mCameras.size(); i++)
    {
        AVT::VmbAPI::CameraPtr cam = mCameras[i]->avt_camera;

        AVT::VmbAPI::FeaturePtr feature;

        AVT::VmbAPI::IFrameObserverPtr observer;

        ok = ok && ( VmbErrorSuccess == cam->Open(VmbAccessModeFull) );

        ok = ok && ( VmbErrorSuccess == cam->GetFeatureByName("TriggerSource", feature) ) && ( VmbErrorSuccess == feature->SetValue("Software") );

        ok = ok && ( VmbErrorSuccess == cam->GetFeatureByName("PixelFormat", feature) ) && ( VmbErrorSuccess == feature->SetValue("BGR8Packed") );

        ok = ok && ( VmbErrorSuccess == cam->GetFeatureByName("PayloadSize", feature) ) && ( VmbErrorSuccess == feature->GetValue(mCameras[i]->payload_size) );

        ok = ok && ( VmbErrorSuccess == cam->GetFeatureByName("GevTimestampTickFrequency", feature) ) && ( VmbErrorSuccess == feature->GetValue(mCameras[i]->tick_frequency) );

        if(ok)
        {
            observer.reset(new FrameObserver(mCameras[i]));

            mCameras[i]->frames.resize(3);

            for( AVT::VmbAPI::FramePtrVector::iterator it = mCameras[i]->frames.begin(); ok && it != mCameras[i]->frames.end(); it++ )
            {
                it->reset(new AVT::VmbAPI::Frame(mCameras[i]->payload_size));
                ok = ok && ( VmbErrorSuccess == (**it).RegisterObserver(observer) );
                ok = ok && ( VmbErrorSuccess == cam->AnnounceFrame(*it) );
            }
        }

        ok = ok && ( VmbErrorSuccess == cam->StartCapture() );

        for( AVT::VmbAPI::FramePtrVector::iterator it = mCameras[i]->frames.begin(); ok && it != mCameras[i]->frames.end(); it++ )
        {
            ok = ok && ( VmbErrorSuccess == cam->QueueFrame(*it) );
        }

        ok = ok && ( VmbErrorSuccess == cam->GetFeatureByName("AcquisitionStart", feature) ) && ( VmbErrorSuccess == feature->RunCommand() );
    }

    if(ok == false)
    {
        std::cerr << "Failed to open the camera!" << std::endl;
    }

    return ok;
}

void AvtRig::close()
{
    bool ok = true;

    std::cerr << "Closing the camera." << std::endl;

    for(size_t i=0; ok && i<mCameras.size(); i++)
    {
        AVT::VmbAPI::FeaturePtr feature;

        AVT::VmbAPI::CameraPtr cam = mCameras[i]->avt_camera;

        ok = ok && ( VmbErrorSuccess == cam->GetFeatureByName("AcquisitionStop", feature) ) && ( VmbErrorSuccess == feature->RunCommand() );

        ok = ok && ( VmbErrorSuccess == cam->EndCapture() );

        ok = ok && ( VmbErrorSuccess == cam->FlushQueue() );

        ok = ok && ( VmbErrorSuccess == cam->RevokeAllFrames() );

        //ok = ok && ( VmbErrorSuccess == mCameras[i]->frame->UnregisterObserver() );

        ok = ok && ( VmbErrorSuccess == cam->Close() );

        mCameras[i]->frames.clear();
    }

    if( ok == false )
    {
        std::cerr << "Error while closing the camera!" << std::endl;
    }
}

void AvtRig::trigger()
{
    /*
    for(size_t i=0; i<mCameras.size(); i++)
    {
        mCameras[i]->avt_camera->FlushQueue();
        mCameras[i]->avt_camera->QueueFrame( mCameras[i]->frame );
    } 
    */

    bool ok = true;

    for(size_t i=0; i<mCameras.size(); i++)
    {
        AVT::VmbAPI::FeaturePtr feature;

        ok = ok && ( mCameras[i]->avt_camera->GetFeatureByName("TriggerSoftware", feature) == VmbErrorSuccess );
        ok = ok && ( feature->RunCommand() == VmbErrorSuccess );
    }

    if(ok == false)
    {
        std::cerr << "Error while triggering the camera!" << std::endl;
    }
}

void AvtRig::read(Image& image)
{
    const size_t N = mCameras.size();

    bool ok = true;

    std::chrono::time_point< std::chrono::steady_clock > until = std::chrono::steady_clock::now() + std::chrono::milliseconds(1000);

    std::vector<Image> images(N);

    for(size_t i=0; ok && i<N; i++)
    {
        std::unique_lock<std::mutex> lock(mCameras[i]->mutex);

        std::cv_status status = mCameras[i]->condition.wait_until(lock, until);

        if( status == std::cv_status::timeout )
        {
            ok = false;
        }
        else
        {
            images[i] = std::move(mCameras[i]->new_image);
            mCameras[i]->new_image.setInvalid();
        }
    }

    if(ok)
    {
        const double timestamp = images.front().getTimestamp();

        std::vector<cv::Mat> frames(N);
        std::transform( images.begin(), images.end(), frames.begin(), [] (Image& im) { return im.getFrame(); } );

        /*
        for(size_t i=0; i<N; i++)
        {
            frames[i] = images[i].getFrame();
        }
        */

        image.setValid(timestamp, frames);
    }
    else
    {
        image.setInvalid();
    }
}

int AvtRig::getNumberOfCameras()
{
    return mCameras.size();
}


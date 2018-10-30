#include <iostream>
#include "AvtRig.h"

//#define AVT_RIG_ONLY_ONE_FRAME

class AvtRig::CameraData
{
public:
    
    CameraData()
    {
        //std::cerr << "CameraData created!" << std::endl;
    }

    ~CameraData()
    {
        //std::cerr << "CameraData deleted!" << std::endl;
    }

    AvtRig* rig;

    AVT::VmbAPI::CameraPtr avt_camera;
    AVT::VmbAPI::FramePtrVector frames;

    bool received;
    Image last_image;

    VmbInt64_t tick_frequency;
    VmbInt64_t payload_size;
};

class AvtRig::FrameObserver : public AVT::VmbAPI::IFrameObserver
{
public:

    FrameObserver(CameraDataPtr cam) : IFrameObserver( cam->avt_camera )
    {
        mCamera = cam;
        //std::cerr << "FrameObserver created!" << std::endl;
    }

    ~FrameObserver()
    {
        //std::cerr << "FrameObserver deleted!" << std::endl;
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
                std::lock_guard<std::mutex> lock(mCamera->rig->mMutex);

                if(mCamera->received)
                {
                    std::cerr << "Received the frame twice!" << std::endl;
                }

                mCamera->last_image = std::move(image);
                mCamera->received = true;

                if( std::all_of(mCamera->rig->mCameras.begin(), mCamera->rig->mCameras.end(), [] (CameraDataPtr cam) { return cam->received; }) )
                {
                    mCamera->rig->mCondition.notify_one();
                }
            }
        }
        else
        {
            std::cerr << "Incorrect frame received!" << std::endl;
        }

#ifndef AVT_RIG_ONLY_ONE_FRAME
        m_pCamera->QueueFrame(frame);
#endif
    }

protected:

    CameraDataPtr mCamera;
};

AvtRig::AvtRig(std::initializer_list<AVT::VmbAPI::CameraPtr> cameras)
{
    setCameras(cameras);
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

        //std::cout << "Z " << ok << std::endl;

        ok = ok && ( VmbErrorSuccess == cam->GetFeatureByName("AcquisitionMode", feature) );
        ok = ok && ( VmbErrorSuccess == feature->SetValue("Continuous") );

        //std::cout << "A " << ok << std::endl;

        ok = ok && ( VmbErrorSuccess == cam->GetFeatureByName("TriggerSource", feature) );
        ok = ok && ( VmbErrorSuccess == feature->SetValue("Software") );

        //std::cout << "B " << ok << std::endl;

        ok = ok && ( VmbErrorSuccess == cam->GetFeatureByName("PixelFormat", feature) );
        ok = ok && ( VmbErrorSuccess == feature->SetValue("BGR8Packed") );

        //std::cout << "C " << ok << std::endl;

        ok = ok && ( VmbErrorSuccess == cam->GetFeatureByName("PayloadSize", feature) );
        ok = ok && ( VmbErrorSuccess == feature->GetValue(mCameras[i]->payload_size) );

        //std::cout << "D " << ok << std::endl;

        ok = ok && ( VmbErrorSuccess == cam->GetFeatureByName("GevTimestampTickFrequency", feature) );
        ok = ok && ( VmbErrorSuccess == feature->GetValue(mCameras[i]->tick_frequency) );

        //std::cout << "E " << ok << std::endl;

        if(ok)
        {
            observer.reset(new FrameObserver(mCameras[i]));

#ifdef AVT_RIG_ONLY_ONE_FRAME
            mCameras[i]->frames.resize(1);
#else
            mCameras[i]->frames.resize(3);
#endif

            for( AVT::VmbAPI::FramePtrVector::iterator it = mCameras[i]->frames.begin(); ok && it != mCameras[i]->frames.end(); it++ )
            {
                it->reset(new AVT::VmbAPI::Frame(mCameras[i]->payload_size));
                ok = ok && ( VmbErrorSuccess == (**it).RegisterObserver(observer) );
                ok = ok && ( VmbErrorSuccess == cam->AnnounceFrame(*it) );
            }
        }

        //std::cout << "F " << ok << std::endl;

        ok = ok && ( VmbErrorSuccess == cam->StartCapture() );

        //std::cout << "G " << ok << std::endl;

#ifndef AVT_RIG_ONLY_ONE_FRAME
        for( AVT::VmbAPI::FramePtrVector::iterator it = mCameras[i]->frames.begin(); ok && it != mCameras[i]->frames.end(); it++ )
        {
            ok = ok && ( VmbErrorSuccess == cam->QueueFrame(*it) );
        }
#endif

        ok = ok && ( VmbErrorSuccess == cam->GetFeatureByName("AcquisitionStart", feature) );
        ok = ok && ( VmbErrorSuccess == feature->RunCommand() );

        //std::cout << "H " << ok << std::endl;
    }

    if(ok == false)
    {
        std::cerr << "Failed to open the camera!" << std::endl;
    }

    return ok;
}

void AvtRig::close()
{
    std::cerr << "Closing the camera." << std::endl;

    bool global_ok = true;

    for(size_t i=0; i<mCameras.size(); i++)
    {
        bool ok = true;

        AVT::VmbAPI::FeaturePtr feature;

        AVT::VmbAPI::CameraPtr cam = mCameras[i]->avt_camera;

        mCameras[i]->frames.clear();

        ok = ok && ( VmbErrorSuccess == cam->GetFeatureByName("AcquisitionStop", feature) );
        ok = ok && ( VmbErrorSuccess == feature->RunCommand() );

        ok = ok && ( VmbErrorSuccess == cam->EndCapture() );

        ok = ok && ( VmbErrorSuccess == cam->FlushQueue() );

        ok = ok && ( VmbErrorSuccess == cam->RevokeAllFrames() );

        ok = ok && ( VmbErrorSuccess == cam->Close() );

        global_ok = global_ok && ok;
    }

    if( global_ok == false )
    {
        std::cerr << "Error while closing the camera!" << std::endl;
    }
}

void AvtRig::trigger()
{
    {
        std::lock_guard<std::mutex> lock(mMutex);

        for(size_t i=0; i<mCameras.size(); i++)
        {
            mCameras[i]->last_image.setInvalid();
            mCameras[i]->received = false;
#ifdef AVT_RIG_ONLY_ONE_FRAME
            mCameras[i]->avt_camera->FlushQueue();
            mCameras[i]->avt_camera->QueueFrame( mCameras[i]->frames.front() );
#endif
            /*
            for( AVT::VmbAPI::FramePtr frame : mCameras[i]->frames )
            {
                mCameras[i]->avt_camera->QueueFrame( mCameras[i]->frames.front() );
            }
            */
            //
        } 
    }

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
    std::unique_lock<std::mutex> lock(mMutex);

    const size_t N = mCameras.size();

    bool ok = true;

    std::chrono::time_point< std::chrono::steady_clock > until = std::chrono::steady_clock::now() + std::chrono::milliseconds(100);

    auto pred_received = [N,this] ()
    {
        bool received = true;

        for(size_t i=0; received && i<N; i++)
        {
            received = mCameras[i]->last_image.isValid();
        }

        return received;
    };

    auto pred_valid = [N,this] ()
    {
        bool valid = true;

        for(size_t i=0; valid && i<N; i++)
        {
            valid = mCameras[i]->received;
        }

        return valid;
    };

    const bool received = mCondition.wait_until(lock, until, pred_received);

    if( received && pred_valid() )
    {
        const double timestamp = mCameras.front()->last_image.getTimestamp();

        std::vector<cv::Mat> frames(N);
        std::transform( mCameras.begin(), mCameras.end(), frames.begin(), [] (CameraDataPtr& d) { return d->last_image.getFrame(); } );

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

void AvtRig::setCameras(std::initializer_list<AVT::VmbAPI::CameraPtr> cameras)
{
    mCameras.clear();
    mCameras.reserve(cameras.size());

    for(AVT::VmbAPI::CameraPtr cam : cameras)
    {
        CameraDataPtr d(new CameraData());

        d->avt_camera = cam;
        d->rig = this;

        mCameras.push_back(d);
    }
}


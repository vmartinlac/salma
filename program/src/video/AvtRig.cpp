#include <iostream>
#include "AvtRig.h"

class AvtRig::CameraData
{
public:

    AvtRig* rig;

    AVT::VmbAPI::CameraPtr avt_camera;
    AVT::VmbAPI::FramePtrVector frames;

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
    }

    void FrameReceived(const AVT::VmbAPI::FramePtr frame) override
    {
        VmbUchar_t* buffer;
        VmbUint64_t timestamp;
        VmbUint32_t width;
        VmbUint32_t height;
        VmbFrameStatusType status;

        std::cout << "callback" << std::endl;

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
                mCamera->last_image = std::move(image);

                if( std::all_of(mCamera->rig->mCameras.begin(), mCamera->rig->mCameras.end(), [] (CameraDataPtr cam) { return cam->last_image.isValid(); }) )
                {
                    mCamera->rig->mCondition.notify_one();
                }
            }
        }
        else
        {
            std::cerr << "Incorrect frame received!" << std::endl;
        }

        //m_pCamera->QueueFrame(frame);
    }

protected:

    CameraDataPtr mCamera;
};

AvtRig::AvtRig()
{
}

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

        ok = ok && ( VmbErrorSuccess == cam->GetFeatureByName("AcquisitionMode", feature) );
        ok = ok && ( VmbErrorSuccess == feature->SetValue("Continuous") );

        ok = ok && ( VmbErrorSuccess == cam->GetFeatureByName("TriggerSource", feature) );
        ok = ok && ( VmbErrorSuccess == feature->SetValue("Software") );

        ok = ok && ( VmbErrorSuccess == cam->GetFeatureByName("PixelFormat", feature) );
        ok = ok && ( VmbErrorSuccess == feature->SetValue("BGR8Packed") );

        ok = ok && ( VmbErrorSuccess == cam->GetFeatureByName("PayloadSize", feature) );
        ok = ok && ( VmbErrorSuccess == feature->GetValue(mCameras[i]->payload_size) );

        ok = ok && ( VmbErrorSuccess == cam->GetFeatureByName("GevTimestampTickFrequency", feature) );
        ok = ok && ( VmbErrorSuccess == feature->GetValue(mCameras[i]->tick_frequency) );

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
            //ok = ok && ( VmbErrorSuccess == cam->QueueFrame(*it) );
        }

        ok = ok && ( VmbErrorSuccess == cam->GetFeatureByName("AcquisitionStart", feature) );
        ok = ok && ( VmbErrorSuccess == feature->RunCommand() );
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
    std::cout << "trigger" << std::endl;

    {
        std::lock_guard<std::mutex> lock(mMutex);

        for(size_t i=0; i<mCameras.size(); i++)
        {
            mCameras[i]->last_image.setInvalid();
            // TODO
            //
            mCameras[i]->avt_camera->FlushQueue();
            mCameras[i]->avt_camera->QueueFrame( mCameras[i]->frames.front() );
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

    std::cout << "read" << std::endl;

    bool ok = true;

    std::chrono::time_point< std::chrono::steady_clock > until = std::chrono::steady_clock::now() + std::chrono::milliseconds(1000);

    auto pred = [N,this] ()
    {
        bool finished = true;

        for(size_t i=0; finished && i<N; i++)
        {
            finished = mCameras[i]->last_image.isValid();
        }

        return finished;
    };

    const bool valid = mCondition.wait_until(lock, until, pred);

    if(valid)
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


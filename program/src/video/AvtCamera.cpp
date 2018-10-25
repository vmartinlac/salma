#include <iostream>
#include "AvtCamera.h"

class AvtCamera::FrameObserver : public AVT::VmbAPI::IFrameObserver
{
public:

    FrameObserver(AvtCamera* camera) : IFrameObserver(camera->mCamera)
    {
        mCamera = camera;
        mCount = 0;
    }

    void FrameReceived(const AVT::VmbAPI::FramePtr frame) override
    {
        //std::lock_guard<std::mutex> lock(mMutex);

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

            cv::Mat wrapper(
                cv::Size(width, height),
                CV_8UC3,
                buffer);

            mCamera->mMutex.lock();

            const double t = double(timestamp) / double(mCamera->mTickFrequency);
            //std::cout << t << std::endl;
            mCamera->mNewImage.setValid(t, wrapper.clone());

            mCamera->mMutex.unlock();
        }

        m_pCamera->QueueFrame(frame);
    }

protected:

    int mCount;
    AvtCamera* mCamera;
    //std::mutex mMutex;
};

AvtCamera::AvtCamera(AVT::VmbAPI::CameraPtr camera)
{
    mCamera = camera;
    mIsOpen = false;
}

AvtCamera::~AvtCamera()
{
    if( mIsOpen )
    {
        close();
    }
}

bool AvtCamera::open()
{
    VmbInt64_t payload_size = 0;
    bool ok = true;

    if( mIsOpen == false )
    {
        if(ok)
        {
            ok = (VmbErrorSuccess == mCamera->Open(VmbAccessModeFull));
        }

        if(ok)
        {
            AVT::VmbAPI::FeaturePtr feature;
            ok = ( VmbErrorSuccess == mCamera->GetFeatureByName("GevTimestampTickFrequency", feature) && VmbErrorSuccess == feature->GetValue(mTickFrequency) );
        }

        if(ok)
        {
            AVT::VmbAPI::FeaturePtr feature;
            ok = ( VmbErrorSuccess == mCamera->GetFeatureByName("TriggerSource", feature) && VmbErrorSuccess == feature->SetValue("FreeRun") );
        }

        if(ok)
        {
            AVT::VmbAPI::FeaturePtr feature;
            ok = ( VmbErrorSuccess == mCamera->GetFeatureByName("PixelFormat", feature) && VmbErrorSuccess == feature->SetValue("BGR8Packed") );
        }

        if(ok)
        {
            ok = ( VmbErrorSuccess == mCamera->StartContinuousImageAcquisition(3, AVT::VmbAPI::IFrameObserverPtr(new FrameObserver(this))) );
        }
    }
    else
    {
        std::cerr << "Tried to open an already open camera!" << std::endl;
    }

    if(ok)
    {
        mIsOpen = true;
    }
    else
    {
        //throw std::runtime_error("Error while starting acquisition from the camera");
        std::cerr << "Could not open the camera!" << std::endl;
        close();
    }

    return ok;
}

void AvtCamera::close()
{
    if( mIsOpen )
    {
        mCamera->StopContinuousImageAcquisition();
        mCamera->Close();
    }
}

void AvtCamera::read(Image& image)
{
    mMutex.lock();
    image = mNewImage;
    mNewImage.setInvalid();
    mMutex.unlock();
}

void AvtCamera::trigger()
{
    /*
    AVT::VmbAPI::FeaturePtr feature;

    bool ok = true;

    ok = ok && ( mCamera->GetFeatureByName("TriggerSoftware", feature) == VmbErrorSuccess );
    ok = ok && ( feature->RunCommand() == VmbErrorSuccess );

    if(ok == false)
    {
        std::cerr << "Failed to trigger the camera!" << std::endl;
    }
    */
}

std::string AvtCamera::getHumanName()
{
    std::string camera_id;
    std::string interface_id;

    bool ok = ( VmbErrorSuccess == mCamera->GetID(camera_id) && VmbErrorSuccess == mCamera->GetInterfaceID(interface_id) );

    if(ok)
    {
        return camera_id + " on " + interface_id;
    }
    else
    {
        return "UNNAMED_CAMERA";
    }
}


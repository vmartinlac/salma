#include <iostream>
#include <thread>
#include <opencv2/core.hpp>
#include "GenICamRig.h"
#include "GenICamCamera.h"

//#define GRAYSCALE

///////////////////////
/*
#include <chrono>
#include <mutex>

void LOG(const char* txt)
{
    static std::mutex mutex;
    const std::chrono::steady_clock::time_point t = std::chrono::steady_clock::now();

    static std::chrono::steady_clock::time_point t0;
    static bool first = true;

    if(first)
    {
        t0 = t;
        first = false;
    }

    const int dt = std::chrono::duration_cast<std::chrono::milliseconds>(t - t0).count();

    mutex.lock();
    std::cout << "t = " << dt << ": " << txt << std::endl;
    mutex.unlock();
}
*/
//////////////////

extern "C" void GenICamCallback(void* user_data, ArvStreamCallbackType type, ArvBuffer* buffer)
{
    if(type = ARV_STREAM_CALLBACK_TYPE_BUFFER_DONE)
    {
        GenICamCamera* cam = static_cast<GenICamCamera*>(user_data);
        cam->onFrameReceived(buffer);
    }
}

void GenICamCamera::onFrameReceived(ArvBuffer* buffer)
{
    //LOG("frame received");
    mMutex.lock();

    const void* buffer_data = nullptr;
    gint width;
    gint height;
    Image image;
    bool ok = true;
    
    if(ok)
    {
        ok = (buffer != nullptr);

        //if(ok == false) std::cout << "Buffer is nullptr!" << std::endl;
    }

    if(ok)
    {
        const int status = arv_buffer_get_status(buffer);
        ok = ( status == ARV_BUFFER_STATUS_SUCCESS );

        //if(ok == false) std::cout << "Incorrect status " << arv_buffer_get_status(buffer) << std::endl;

        //LOG( std::to_string(arv_buffer_get_frame_id(buffer)).c_str() );
        /*
        switch(status)
        {
        case ARV_BUFFER_STATUS_UNKNOWN:
            std::cout << "UNKNOWN" << std::endl;
            break;
        case ARV_BUFFER_STATUS_SUCCESS:
            std::cout << "SUCCESS" << std::endl;
            break;
        case ARV_BUFFER_STATUS_CLEARED:
            std::cout << "CLEARED" << std::endl;
            break;
        case ARV_BUFFER_STATUS_TIMEOUT:
            std::cout << "TIMEOUT" << std::endl;
            break;
        case ARV_BUFFER_STATUS_MISSING_PACKETS:
            std::cout << "MISSING_PACKETS" << std::endl;
            break;
        case ARV_BUFFER_STATUS_WRONG_PACKET_ID:
            std::cout << "WRONG_PACKET_ID" << std::endl;
            break;
        case ARV_BUFFER_STATUS_SIZE_MISMATCH:
            std::cout << "SIZE_MISMATCH" << std::endl;
            break;
        case ARV_BUFFER_STATUS_FILLING:
            std::cout << "FILLING" << std::endl;
            break;
        case ARV_BUFFER_STATUS_ABORTED:
            std::cout << "ABORTED" << std::endl;
            break;
        }
        */
    }

    if(ok)
    {
        ok = ( arv_buffer_get_payload_type(buffer) == ARV_BUFFER_PAYLOAD_TYPE_IMAGE );
        //if(ok == false) std::cout << "Incorrect payload type " << arv_buffer_get_payload_type(buffer) << std::endl;
    }

    if(ok)
    {
        arv_buffer_get_image_region(buffer, nullptr, nullptr, &width, &height);
        ok = (width > 0 && height > 0);
    }

    if(ok)
    {
        buffer_data = arv_buffer_get_data(buffer, nullptr);
        ok = bool(buffer_data);
    }

    if(ok)
    {
        const guint64 raw_timestamp = arv_buffer_get_system_timestamp(buffer);

        if(mFirstFrame)
        {
            mFirstFrame = false;
            mFirstTimestamp = raw_timestamp;
        }

        const double timestamp = double(raw_timestamp - mFirstTimestamp) * 1.0e-9;

#ifdef GRAYSCALE
        cv::Mat frame( cv::Size(width, height), CV_8UC1 );
#else
        cv::Mat frame( cv::Size(width, height), CV_8UC3 );
#endif

        std::copy(
            static_cast<const uint8_t*>(buffer_data),
#ifdef GRAYSCALE
            static_cast<const uint8_t*>(buffer_data) + width*height,
#else
            static_cast<const uint8_t*>(buffer_data) + width*height*3,
#endif
            frame.ptr(0));

        image.setValid(timestamp, frame);
    }

    if(ok)
    {
        mLastImage = std::move(image);
    }

    if(buffer)
    {
        //arv_stream_push_buffer(mStream, buffer);
        mAvailableBuffers.push_back(buffer);
    }

    mMutex.unlock();

    if(ok)
    {
        mRig->onFrameReceived();
    }

    //if(ok == false) std::cerr << "FRAME REJECTED!" << std::endl;
}

void GenICamCamera::takeLastImage(Image& image)
{
    mMutex.lock();
    image = std::move(mLastImage);
    mMutex.unlock();
}

GenICamCamera::GenICamCamera(GenICamRig* rig, const std::string& id)
{
    mIsOpen = false;
    mRig = rig;
    mId = id;
    mDevice = nullptr;
    mStream = nullptr;
    mFirstFrame = true;
}

GenICamCamera::~GenICamCamera()
{
    if(mIsOpen)
    {
        close();
    }
}

/*
ArvStream* GenICamCamera::getArvStream()
{
    return mStream;
}
*/

std::string GenICamCamera::getId()
{
    return mId;
}

void GenICamCamera::prepareTrigger()
{
    mMutex.lock();
    mLastImage.setInvalid();
    if(mAvailableBuffers.empty() == false)
    {
        arv_stream_push_buffer(mStream, mAvailableBuffers.back());
        mAvailableBuffers.pop_back();
    }
    mMutex.unlock();
}

void GenICamCamera::softwareTrigger()
{
    arv_device_execute_command(mDevice, "TriggerSoftware");
}

bool GenICamCamera::open(bool external_trigger)
{
    if(mIsOpen) throw std::runtime_error("camera is already open");

    mFirstFrame = true;

    mIsOpen = true;

    if(mIsOpen)
    {
        mDevice = arv_open_device(mId.c_str());
        mIsOpen = bool(mDevice);
    }

    if(mIsOpen)
    {
        mStream = arv_device_create_stream(mDevice, GenICamCallback, this);
        mIsOpen = bool(mStream);

    }

    if(mIsOpen)
    {
        //g_object_set(mStream, "packet-timeout", 1000000, nullptr);
        //g_object_set(mStream, "frame-retention", 1000000, nullptr);
    }

    if(mIsOpen)
    {
        if(external_trigger)
        {
            arv_device_set_string_feature_value(mDevice, "TriggerSource", "Line1");
        }
        else
        {
            arv_device_set_string_feature_value(mDevice, "TriggerSource", "Software");
        }
        mIsOpen = ( arv_device_get_status(mDevice) == ARV_DEVICE_STATUS_SUCCESS );
    }

    if(mIsOpen)
    {
        arv_device_set_string_feature_value(mDevice, "AcquisitionMode", "Continuous");
        mIsOpen = ( arv_device_get_status(mDevice) == ARV_DEVICE_STATUS_SUCCESS );
    }

    if(mIsOpen)
    {
#ifdef GRAYSCALE
        arv_device_set_string_feature_value(mDevice, "PixelFormat", "Mono8");
#else
        arv_device_set_string_feature_value(mDevice, "PixelFormat", "BGR8Packed");
#endif
        mIsOpen = ( arv_device_get_status(mDevice) == ARV_DEVICE_STATUS_SUCCESS );
    }

    /*
    if(mIsOpen)
    {
        mClockFrequency = arv_device_get_integer_feature_value(mDevice, "GevTimestampTickFrequency");
        mIsOpen = ( arv_device_get_status(mDevice) == ARV_DEVICE_STATUS_SUCCESS );
    }
    */

    if(mIsOpen)
    {
        arv_device_execute_command(mDevice, "GVSPAdjustPacketSize");
        mIsOpen = ( arv_device_get_status(mDevice) == ARV_DEVICE_STATUS_SUCCESS );
    }

    if(mIsOpen)
    {
        mPayload = arv_device_get_integer_feature_value(mDevice, "PayloadSize");
        mIsOpen = ( arv_device_get_status(mDevice) == ARV_DEVICE_STATUS_SUCCESS );
    }

    if(mIsOpen)
    {
        /*
        const int num_buffers = 4;

        for(int i=0; i<num_buffers; i++)
        {
            ArvBuffer* buffer = arv_buffer_new(mPayload, nullptr);
            arv_stream_push_buffer(mStream, buffer);
        }
        */
        
        mAvailableBuffers.push_back( arv_buffer_new(mPayload, nullptr) );
        //arv_stream_push_buffer(mStream, mBuffer);
    }

    if(mIsOpen)
    {
        arv_device_execute_command(mDevice, "AcquisitionStart");
        mIsOpen = ( arv_device_get_status(mDevice) == ARV_DEVICE_STATUS_SUCCESS );
    }

    return mIsOpen;
}

void GenICamCamera::close()
{
    if(mIsOpen)
    {
        arv_device_execute_command(mDevice, "AcquisitionStop");
        mIsOpen = false;
        g_clear_object(&mDevice);
    }
}


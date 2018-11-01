#include <iostream>
#include <opencv2/core.hpp>
#include "GenICamRig.h"
#include "GenICamCamera.h"

/*
extern "C" void GenICamCallback(void* user_data, ArvStreamCallbackType type, ArvBuffer* buffer)
{
    GenICamCamera* cam = static_cast<GenICamCamera*>(user_data);

    std::cout << "callback " << type << std::endl;

    if(type == ARV_STREAM_CALLBACK_TYPE_BUFFER_DONE)
    {
        arv_stream_push_buffer(cam->getArvStream(), buffer);
    }
}
*/

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

std::string GenICamCamera::getId()
{
    return mId;
}

void GenICamCamera::trigger()
{
    arv_device_execute_command(mDevice, "TriggerSoftware");
}

bool GenICamCamera::open()
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
        mStream = arv_device_create_stream(mDevice, nullptr, nullptr);
        //mStream = arv_device_create_stream(mDevice, GenICamCallback, this);
        mIsOpen = bool(mStream);
    }

    if(mIsOpen)
    {
        arv_device_set_string_feature_value(mDevice, "TriggerSource", "Software");
        mIsOpen = ( arv_device_get_status(mDevice) == ARV_DEVICE_STATUS_SUCCESS );
    }

    if(mIsOpen)
    {
        arv_device_set_string_feature_value(mDevice, "AcquisitionMode", "Continuous");
        mIsOpen = ( arv_device_get_status(mDevice) == ARV_DEVICE_STATUS_SUCCESS );
    }

    if(mIsOpen)
    {
        arv_device_set_string_feature_value(mDevice, "PixelFormat", "BGR8Packed");
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
        mPayload = arv_device_get_integer_feature_value(mDevice, "PayloadSize");
        mIsOpen = ( arv_device_get_status(mDevice) == ARV_DEVICE_STATUS_SUCCESS );
    }

    if(mIsOpen)
    {
        const int num_buffers = 4;

        for(int i=0; i<num_buffers; i++)
        {
            ArvBuffer* buffer = arv_buffer_new(mPayload, nullptr);
            arv_stream_push_buffer(mStream, buffer);
        }
        
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
        g_clear_object(&mStream);
        g_clear_object(&mDevice);
    }
}

void GenICamCamera::read(std::chrono::time_point<std::chrono::steady_clock> expiration_time, Image& image)
{
    ArvBuffer* buffer = nullptr;
    int duration = 0;
    const void* data = nullptr;
    gint width;
    gint height;
    bool ok = true;

    image.setInvalid();

    if(ok)
    {
        duration = std::chrono::duration_cast< std::chrono::milliseconds >( expiration_time - std::chrono::steady_clock::now() ).count();
        ok = (duration > 0);
    }

    if(ok)
    {
        buffer = arv_stream_timeout_pop_buffer(mStream, 1000*duration);
        ok = bool(buffer);
    }

    if(ok)
    {
        ok = (arv_buffer_get_status(buffer) == ARV_BUFFER_STATUS_SUCCESS);
    }

    if(ok)
    {
        ok = (arv_buffer_get_payload_type(buffer) == ARV_BUFFER_PAYLOAD_TYPE_IMAGE);
    }

    if(ok)
    {
        arv_buffer_get_image_region(buffer, nullptr, nullptr, &width, &height);
        ok = (width > 0 && height > 0);
    }

    if(ok)
    {
        data = arv_buffer_get_data(buffer, nullptr);
        ok = bool(data);
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

        cv::Mat frame(
            cv::Size(width, height),
            CV_8UC3);

        std::copy(
            static_cast<const uint8_t*>(data),
            static_cast<const uint8_t*>(data) + width*height*3,
            frame.ptr(0));

        image.setValid(timestamp, frame);
    }

    if(buffer)
    {
        arv_stream_push_buffer(mStream, buffer);
    }
}


#include "Camera.h"
#include "Rig.h"

#define NUM_BUFFERS 12
#define MAX_OUT_OF_STREAM_BUFFERS 4

Camera::Camera(Rig* rig, const std::string& id, int rank) :
    mRig(rig),
    mRank(rank),
    mId(id)
{
}

void Camera::open()
{
    gint64 payload = 0;
    bool ok = true;

    if(ok)
    {
        mDevice = arv_open_device(mId.c_str());
        if(mDevice == nullptr) throw std::runtime_error("could not open device");
        ok = (mDevice != nullptr);
    }

    if(ok)
    {
        arv_device_set_string_feature_value(mDevice, "PixelFormat", "BGR8Packed");
        ok = ( arv_device_get_status(mDevice) == ARV_DEVICE_STATUS_SUCCESS );
    }

    if(ok)
    {
        arv_device_set_string_feature_value(mDevice, "TriggerSource", "Freerun");
        ok = ( arv_device_get_status(mDevice) == ARV_DEVICE_STATUS_SUCCESS );
    }

    if(ok)
    {
        mStream = arv_device_create_stream(mDevice, stream_callback, this);
        if(mStream == nullptr) throw std::runtime_error("could not open stream");
    }

    if(ok)
    {
        payload = arv_device_get_integer_feature_value(mDevice, "PayloadSize");
        ok = ( arv_device_get_status(mDevice) == ARV_DEVICE_STATUS_SUCCESS );
    }

    if(ok)
    {
        for(int i=0; i<NUM_BUFFERS; i++)
        {
            ArvBuffer* buffer = arv_buffer_new_allocate(payload);
            arv_stream_push_buffer(mStream, buffer);
        }
    }

    arv_device_execute_command(mDevice, "AcquisitionStart");
}

void Camera::close()
{
    arv_device_execute_command(mDevice, "AcquisitionStop");
    g_object_unref(mDevice);
}

void Camera::stream_callback(void* user_data, ArvStreamCallbackType type, ArvBuffer* buffer)
{
    bool ok = true;

    if(ok)
    {
        ok = ( type == ARV_STREAM_CALLBACK_TYPE_BUFFER_DONE ) && bool(buffer);
    }

    if(ok)
    {
        ok = ( arv_buffer_get_payload_type(buffer) == ARV_BUFFER_PAYLOAD_TYPE_IMAGE );
    }

    if(ok)
    {
        const guint32 id = arv_buffer_get_frame_id(buffer);

        Camera* cam = reinterpret_cast<Camera*>(user_data);

        cam->mMutex.lock();
        cam->mBuffers[id] = buffer;
        //arv_stream_push_buffer(cam->mStream, buffer);
        cam->mMutex.unlock();
        cam->mRig->mConditionVariable.notify_one();
        std::cout << "Callback" << std::endl;
    }
}

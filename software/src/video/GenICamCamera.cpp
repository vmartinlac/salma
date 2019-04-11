#include "GenICamRig.h"
#include "GenICamCamera.h"

GenICamCamera::GenICamCamera(GenICamRig* rig, const std::string& id, int rank) :
    mRig(rig),
    mRank(rank),
    mId(id)
{
}

std::string GenICamCamera::getId()
{
    return mId;
}

bool GenICamCamera::open(bool software_trigger)
{
    gint64 payload = 0;
    bool ok = true;
    const char* err = "";

    mTab1.clear();
    mTab2.clear();

    if(ok)
    {
        mCamera = arv_camera_new(mId.c_str());
        ok = bool(mCamera);
        err = "Could not open camera!";
    }

    if(ok)
    {
        mDevice = arv_camera_get_device(mCamera);
        ok = bool(mDevice);
        err = "Could not get device from camera!";
    }

    if(ok)
    {
        arv_device_set_string_feature_value(mDevice, "PixelFormat", "BGR8Packed");
        ok = ( arv_device_get_status(mDevice) == ARV_DEVICE_STATUS_SUCCESS );
        err = "Could not set pixelformat!";
    }

    if(ok)
    {
        if(software_trigger)
        {
            arv_device_set_string_feature_value(mDevice, "TriggerSource", "Software");
        }
        else
        {
            arv_device_set_string_feature_value(mDevice, "TriggerSource", "Line1");
        }
        ok = ( arv_device_get_status(mDevice) == ARV_DEVICE_STATUS_SUCCESS );
        err = "Could not set trigger source!";
    }

    if(ok)
    {
        payload = arv_device_get_integer_feature_value(mDevice, "PayloadSize");
        ok = ( arv_device_get_status(mDevice) == ARV_DEVICE_STATUS_SUCCESS );
        err = "Could not get payload size!";
    }

    if(ok)
    {
        mStream = arv_camera_create_stream(mCamera, nullptr, this);
        ok = bool(mStream);
        err = "Could not create stream!";
    }

    if(ok)
    {
        g_signal_connect(mStream, "new-buffer", G_CALLBACK(stream_callback), this);
        arv_stream_set_emit_signals(mStream, true);
    }


    if(ok)
    {
        for(int i=0; i<GENICAM_NUM_BUFFERS; i++)
        {
            ArvBuffer* const buffer = arv_buffer_new_allocate(payload);
            arv_stream_push_buffer(mStream, buffer);
        }
    }

    if(ok)
    {
        arv_camera_start_acquisition(mCamera);
    }

    if(ok == false)
    {
        std::cout << err << std::endl;
        throw;
    }

    return ok;
}

void GenICamCamera::close()
{
    arv_camera_stop_acquisition(mCamera);

    if(mStream)
    {
        arv_stream_set_emit_signals(mStream, false);

        g_object_unref(mStream);
        mStream = nullptr;
    }

    if(mCamera)
    {
        g_object_unref(mCamera);
        mCamera = nullptr;
    }

    {
        std::array<ArvBuffer*, GENICAM_NUM_BUFFERS+1> tmp;
        std::fill(tmp.begin(), tmp.end(), nullptr);
        mTab1.take(tmp.begin());
        for(auto it = tmp.begin(); *it!=nullptr; it++)
        {
            g_object_unref(*it);
            it++;
        }
    }

    {
        for(std::pair<guint32,ArvBuffer*> p : mTab2)
        {
            g_object_unref(p.second);
        }
        mTab2.clear();
    }
}

void GenICamCamera::stream_callback(ArvStream *stream, void *user_data)
{
    GenICamCamera* const cam = reinterpret_cast<GenICamCamera*>(user_data);
    ArvBuffer* const buffer = arv_stream_try_pop_buffer (stream);
    bool ok = true;

    if(ok)
    {
        ok = bool(cam) && bool(cam->mRig);
    }

    if(ok)
    {
        ok = bool(buffer) && (arv_buffer_get_status(buffer) == ARV_BUFFER_STATUS_SUCCESS);
    }

    if(ok)
    {
        ok = ( arv_buffer_get_payload_type(buffer) == ARV_BUFFER_PAYLOAD_TYPE_IMAGE );
    }

    if(ok)
    {
        ArvBuffer* const to_push = cam->mTab1.push(buffer);

        if(to_push)
        {
            arv_stream_push_buffer(cam->mStream, to_push);
        }

        cam->mRig->signalImageAvailability();
    }
}

void GenICamCamera::softwareTrigger()
{
    arv_device_execute_command(mDevice, "TriggerSoftware");
    //const bool ok = ( arv_device_get_status(mDevice) == ARV_DEVICE_STATUS_SUCCESS );
    //if(ok == false) std::cerr << "Software trigger failed!" << std::endl;
}


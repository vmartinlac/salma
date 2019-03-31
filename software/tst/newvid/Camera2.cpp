#include "Camera2.h"
#include "Rig2.h"

void Camera::callback(const VmbHandle_t handle, VmbFrame_t*  frame)
{
    Camera* cam = reinterpret_cast<Camera*>(frame->context[0]);

    if(cam)
    {
        std::cout << "Frame received on camera " << cam->mRank << std::endl;
    }
}

Camera::Camera(Rig* rig, const std::string& id, int rank) :
    mRig(rig),
    mRank(rank),
    mId(id),
    mIsOpen(false)
{
}

void Camera::open()
{
    const char* err = "";
    bool ok = true;
    VmbInt64_t payload = 0;
    const std::string pixelformat("BGR8Packed");

    if(ok)
    {
        ok = ( VmbErrorSuccess == VmbCameraInfoQuery(mId.c_str(), &mInfo, sizeof(VmbCameraInfo_t)) );
        err = "Could not query camera info!";
    }

    if(ok)
    {
        ok = ( VmbErrorSuccess == VmbCameraOpen(mId.c_str(), VmbAccessModeFull, &mHandle) );
        err = "Could not open camera!";
    }

    if(ok)
    {
        ok = ( VmbErrorSuccess == VmbFeatureCommandRun(mHandle, "GVSPAdjustPacketSize") );

        bool done = false;
        while(done == false && ok)
        {
            ok = ( VmbErrorSuccess == VmbFeatureCommandIsDone(mHandle, "GVSPAdjustPacketSize", &done) );
        }
    }

    if(ok)
    {
        ok = ( VmbErrorSuccess == VmbFeatureEnumSet(mHandle, "PixelFormat", pixelformat.c_str()) );
        err = "Could not set pixel format!";
    }

    if(ok)
    {
        const char* pixelformat2 = nullptr;
        ok = ( VmbErrorSuccess == VmbFeatureEnumGet( mHandle, "PixelFormat", &pixelformat2 ) );
        err = "Could not read pixel format!";

        if(ok)
        {
            ok = bool(pixelformat2) && ( pixelformat == pixelformat2 );
            err = "Incorrect pixel format!";
        }
    }

    if(ok)
    {
        ok = ( VmbErrorSuccess == VmbFeatureIntGet(mHandle, "PayloadSize", &payload) );
        err = "Could not read payload size!";
    }

    if(ok)
    {
        mBuffers.resize(GENICAM_NUM_BUFFERS);

        for( size_t i = 0; ok && i<mBuffers.size(); i++ )
        {
            Buffer& b = mBuffers[i];

            b.data.resize(payload);

            b.frame.buffer = &b.data.front();
            b.frame.bufferSize = payload;
            b.frame.context[0] = this;
            b.frame.context[1] = nullptr;
            b.frame.context[2] = nullptr;
            b.frame.context[3] = nullptr;

            ok = ( VmbErrorSuccess == VmbFrameAnnounce( mHandle, &b.frame, sizeof(VmbFrame_t)) );
            err = "Could not announce buffer!";
        }
    }

    if(ok)
    {
        ok = ( VmbErrorSuccess == VmbCaptureStart( mHandle ) );
        err = "Could not start capture!";
    }

    if(ok)
    {
        for(size_t i=0; ok && i<mBuffers.size(); i++)
        {
            Buffer& b = mBuffers[i];

            ok = ( VmbErrorSuccess == VmbCaptureFrameQueue( mHandle, &b.frame, callback ) );
            err = "Could not queue frame!";
        }
    }

    if(ok)
    {
        ok = ( VmbErrorSuccess == VmbFeatureCommandRun(mHandle,"AcquisitionStart") );
        err = "Could not start acquisition!";
    }

    if(ok)
    {
        mIsOpen = true;
    }
    else
    {
        std::cout << err << std::endl;
    }
}

void Camera::close()
{
    bool ok = true;
    const char* err = "";

    if(ok)
    {
        ok = mIsOpen;
        err = "Camera was not open!";
    }

    if(ok)
    {
        ok = ( VmbErrorSuccess == VmbFeatureCommandRun( mHandle,"AcquisitionStop" ) );
        err = "Could not stop acquisition!";
    }

    if(ok)
    {
        ok = ( VmbErrorSuccess == VmbCaptureEnd( mHandle ) );
        err = "Could not stop capture!";
    }

    if(ok)
    {
        ok = ( VmbErrorSuccess == VmbFrameRevokeAll(mHandle) );
        err = "Could not revoke frames!";
    }

    if(ok)
    {
        ok = ( VmbErrorSuccess == VmbCameraClose(mHandle) );
        err = "Could not close camera!";
    }

    if(ok == false)
    {
        std::cout << err << std::endl;
    }
}

void Camera::trigger()
{
}


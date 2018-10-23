#include <iostream>
#include "AvtCamera.h"

AvtCamera::AvtCamera(const VmbCameraInfo_t& infos)
{
    m_camera_id = infos.cameraIdString;
    m_camera_name = infos.cameraName;
    m_camera_model = infos.modelName;
    m_camera_serial = infos.serialString;
    m_camera_permitted_access = infos.permittedAccess;
    m_interface_id = infos.interfaceIdString;
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
    VmbError_t err = VmbErrorSuccess;
    VmbInt64_t payload_size = 0;
    bool ok = true;

    if( mIsOpen == false )
    {
        if(ok)
        {
            err = VmbCameraOpen(m_camera_id.c_str(), VmbAccessModeFull, &mHandle);
            ok = (VmbErrorSuccess == err);
        }

        if(ok)
        {
            err = VmbFeatureIntGet(mHandle, "PayloadSize", &payload_size);
            ok = (VmbErrorSuccess == err);
        }

        if(ok)
        {
            err = VmbFeatureIntGet(mHandle, "GevTimestampTickFrequency", &m_tick_frequency);
            ok = (VmbErrorSuccess == err);
        }

        if(ok)
        {
            err = VmbFeatureEnumSet(mHandle, "AcquisitionMode", "Continuous");
            ok = (VmbErrorSuccess == err);
        }

        if(ok)
        {
            err = VmbFeatureEnumSet(mHandle, "TriggerSource", "Software");
            ok = (VmbErrorSuccess == err);
        }

        if(ok)
        {
            err = VmbFeatureEnumSet(mHandle, "PixelFormat", "BGR8Packed");
            ok = (VmbErrorSuccess == err);
        }

        if(ok)
        {
            mFrames.resize(10);

            for(Frame& f : mFrames)
            {
                f.buffer.resize(payload_size);
                f.vimba_frame.buffer = &f.buffer.front();
                f.vimba_frame.bufferSize = payload_size;
                f.vimba_frame.context[0] = this;
                f.vimba_frame.context[1] = nullptr;
                f.vimba_frame.context[2] = nullptr;
                f.vimba_frame.context[3] = nullptr;

                err = VmbFrameAnnounce(mHandle, &f.vimba_frame, sizeof(VmbFrame_t));
                ok = ok && (VmbErrorSuccess == err);
            }
        }

        if(ok)
        {
            err = VmbCaptureStart(mHandle);
            ok = (VmbErrorSuccess == err);
        }

        if(ok)
        {
            for(Frame& f : mFrames)
            {
                err = VmbCaptureFrameQueue(mHandle, &f.vimba_frame, callback);
                ok = ok && (VmbErrorSuccess == err);
            }
        }

        if(ok)
        {
            err = VmbFeatureCommandRun(mHandle, "AcquisitionStart");
            ok = (VmbErrorSuccess == err);
        }
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
        VmbFeatureCommandRun(mHandle, "AcquisitionStop");

        VmbCaptureEnd( mHandle );

        VmbCaptureQueueFlush( mHandle );

        VmbFrameRevokeAll( mHandle );

        VmbCameraClose( mHandle );

        mFrames.clear();

        mIsOpen = false;
    }
}

/*
void AvtCamera::read(Image& image)
{
    image.setInvalid();

    if( mIsOpen )
    {
        Frame& f = mFrames[m_next_frame];

        VmbError_t err = VmbCaptureFrameWait(mHandle, &f.vimba_frame, m_max_wait_milliseconds);

        if(err == VmbErrorSuccess)
        {
            m_next_frame = (m_next_frame+1) % mFrames.size();

            if(
                (f.vimba_frame.receiveStatus == VmbFrameStatusComplete) &&
                (f.vimba_frame.pixelFormat == VmbPixelFormatBgr8) &&
                (f.vimba_frame.receiveFlags & VmbFrameFlagsTimestamp) &&
                (f.vimba_frame.receiveFlags & VmbFrameFlagsDimension) )
            {
                cv::Mat wrapper(
                    cv::Size(f.vimba_frame.width, f.vimba_frame.height),
                    CV_8UC3,
                    f.vimba_frame.buffer);

                const double timestamp = double(f.vimba_frame.timestamp) / double(m_tick_frequency);

                image.setValid(timestamp, wrapper.clone());
            }
            else
            {
                std::cerr << "Incorrect frame received!" << std::endl;
            }

            VmbCaptureFrameQueue( mHandle, &f.vimba_frame, nullptr);
        }
    }
}
*/

void AvtCamera::read(Image& image)
{
    mMutex.lock();
    image = mNewImage;
    mNewImage.setInvalid();
    mMutex.unlock();
}

void AvtCamera::trigger()
{
    VmbError_t err = VmbFeatureCommandRun(mHandle, "TriggerSoftware");

    if( VmbErrorSuccess != err )
    {
        std::cerr << "Failed to trigger the camera!" << std::endl;
    }
}

std::string AvtCamera::getHumanName()
{
    return m_camera_id + " on " + m_interface_id;
}

void VMB_CALL AvtCamera::callback( const VmbHandle_t handle, VmbFrame_t* frame )
{
    if(
        (VmbFrameStatusComplete == frame->receiveStatus) &&
        (frame->pixelFormat == VmbPixelFormatBgr8) &&
        (frame->receiveFlags & VmbFrameFlagsTimestamp) &&
        (frame->receiveFlags & VmbFrameFlagsDimension) )
    {
        AvtCamera* camera = static_cast<AvtCamera*>(frame->context[0]);

        cv::Mat wrapper(
            cv::Size(frame->width, frame->height),
            CV_8UC3,
            frame->buffer);

        camera->mMutex.lock();

        const double timestamp = double(frame->timestamp) / double(camera->m_tick_frequency);
        camera->mNewImage.setValid(timestamp, wrapper.clone());

        camera->mMutex.unlock();
    }
    else
    {
        std::cout << "error while receiving frame !" << std::endl;
    }

    VmbCaptureFrameQueue( handle, frame, callback );
}


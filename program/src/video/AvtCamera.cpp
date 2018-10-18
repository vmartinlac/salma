#include <iostream>
#include "AvtCamera.h"

AvtCamera::AvtCamera(const VmbCameraInfo_t& infos, TriggerMode trigger_mode)
{
    m_trigger_mode = trigger_mode;
    m_max_wait_milliseconds = 1000;

    m_camera_id = infos.cameraIdString;
    m_camera_name = infos.cameraName;
    m_camera_model = infos.modelName;
    m_camera_serial = infos.serialString;
    m_camera_permitted_access = infos.permittedAccess;
    m_interface_id = infos.interfaceIdString;
    m_is_open = false;
}

AvtCamera::~AvtCamera()
{
    if( m_is_open )
    {
        close();
    }
}

bool AvtCamera::open()
{
    VmbError_t err = VmbErrorSuccess;
    VmbInt64_t payload_size = 0;
    bool ok = true;

    if( m_is_open == false )
    {
        if(ok)
        {
            err = VmbCameraOpen(m_camera_id.c_str(), VmbAccessModeFull, &m_handle);
            ok = (VmbErrorSuccess == err);
        }

        if(ok)
        {
            err = VmbFeatureIntGet(m_handle, "PayloadSize", &payload_size);
            ok = (VmbErrorSuccess == err);
        }

        if(ok)
        {
            err = VmbFeatureIntGet(m_handle, "GevTimestampTickFrequency", &m_tick_frequency);
            ok = (VmbErrorSuccess == err);
        }

        if(ok)
        {
            err = VmbFeatureEnumSet(m_handle, "AcquisitionMode", "Continuous");
            ok = (VmbErrorSuccess == err);
        }

        if(ok)
        {
            err = VmbFeatureEnumSet(m_handle, "TriggerSource", "Software");
            ok = (VmbErrorSuccess == err);
        }

        if(ok)
        {
            err = VmbFeatureEnumSet(m_handle, "PixelFormat", "BGR8Packed");
            ok = (VmbErrorSuccess == err);
        }

        if(ok)
        {
            m_frames.resize(1);

            for(Frame& f : m_frames)
            {
                f.buffer.resize(payload_size);
                f.vimba_frame.buffer = &f.buffer.front();
                f.vimba_frame.bufferSize = payload_size;
                f.vimba_frame.context[0] = nullptr;
                f.vimba_frame.context[1] = nullptr;
                f.vimba_frame.context[2] = nullptr;
                f.vimba_frame.context[3] = nullptr;

                err = VmbFrameAnnounce(m_handle, &f.vimba_frame, sizeof(VmbFrame_t));
                ok = ok && (VmbErrorSuccess == err);
            }
        }

        if(ok)
        {
            err = VmbCaptureStart(m_handle);
            ok = (VmbErrorSuccess == err);
        }

        if(ok)
        {
            m_next_frame = 0;

            for(Frame& f : m_frames)
            {
                err = VmbCaptureFrameQueue(m_handle, &f.vimba_frame, nullptr);
                ok = ok && (VmbErrorSuccess == err);
            }
        }

        if(ok)
        {
            err = VmbFeatureCommandRun(m_handle, "AcquisitionStart");
            ok = (VmbErrorSuccess == err);
        }
    }

    if(ok)
    {
        m_is_open = true;
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
    if( m_is_open )
    {
        VmbFeatureCommandRun(m_handle, "AcquisitionStop");

        VmbCaptureEnd( m_handle );

        VmbCaptureQueueFlush( m_handle );

        VmbFrameRevokeAll( m_handle );

        VmbCameraClose( m_handle );

        m_frames.clear();

        m_is_open = false;
    }
}

void AvtCamera::read(Image& image)
{
    image.setInvalid();

    if( m_is_open )
    {
        Frame& f = m_frames[m_next_frame];

        VmbError_t err = VmbCaptureFrameWait(m_handle, &f.vimba_frame, m_max_wait_milliseconds);

        if(err == VmbErrorSuccess)
        {
            m_next_frame = (m_next_frame+1) % m_frames.size();

            std::cout << f.vimba_frame.receiveStatus << ' ';
            std::cout << (f.vimba_frame.pixelFormat == VmbPixelFormatBgr8) << ' ';
            std::cout << bool(f.vimba_frame.receiveFlags & VmbFrameFlagsTimestamp) << ' ';
            std::cout << bool(f.vimba_frame.receiveFlags & VmbFrameFlagsDimension) << std::endl;
            std::cout << std::endl;

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

            VmbCaptureFrameQueue( m_handle, &f.vimba_frame, nullptr);
        }
    }
}

void AvtCamera::trigger()
{
    VmbError_t err = VmbFeatureCommandRun(m_handle, "TriggerSoftware");

    if( VmbErrorSuccess != err )
    {
        std::cerr << "Failed to trigger the camera!" << std::endl;
    }
}

std::string AvtCamera::getHumanName()
{
    return m_camera_id + " on " + m_interface_id;
}


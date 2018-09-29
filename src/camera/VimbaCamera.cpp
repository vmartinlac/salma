#include <cassert>
#include <iostream>
#include <stdexcept>
#include <mutex>
#include <vector>
#include <opencv2/core.hpp>
#include <VimbaC/Include/VimbaC.h>
#include "VimbaCamera.h"

// declaration of VimbaCamera.

class VimbaCamera : public Camera
{

public:

    VimbaCamera(const VmbCameraInfo_t& infos);

    ~VimbaCamera() override;

    std::string getHumanName() override;

    bool open() override;

    void close() override;

    void read(Image& image) override;

protected:

    //static void VMB_CALL frame_callback( const VmbHandle_t camera, VmbFrame_t* frame );

protected:

    std::string m_camera_id;
    std::string m_camera_name;
    std::string m_camera_model;
    std::string m_camera_serial;
    VmbAccessMode_t m_camera_permitted_access;
    std::string m_interface_id;

    struct Frame
    {
        VmbFrame_t frame;
        std::vector<uint8_t> buffer;
    };

    bool m_is_open;
    VmbHandle_t m_handle;
    VmbInt64_t m_tick_frequency;
    std::vector<Frame> m_frames;
    int m_next_frame;
};

// declaration of VimbaCameraManagerImpl

class VimbaCameraManagerImpl : public VimbaCameraManager
{

public:

    static VimbaCameraManagerImpl& instance() { return m_instance; }

    bool initialize() override;

    void finalize() override;

    CameraPtr getDefaultCamera() override;

    int getNumCameras() override;

    CameraPtr getCamera(int id) override;

protected:

    std::vector< CameraPtr > m_cameras;
    static VimbaCameraManagerImpl m_instance;
};

// VimbaCamera

/*
void VMB_CALL VimbaCamera::frame_callback( const VmbHandle_t handle, VmbFrame_t* frame )
{
    if(
        (VmbFrameStatusComplete == frame->receiveStatus) &&
        (frame->pixelFormat == VmbPixelFormatBgr8) &&
        (frame->receiveFlags & VmbFrameFlagsTimestamp) &&
        (frame->receiveFlags & VmbFrameFlagsDimension) )
    {
        VimbaCamera* camera = static_cast<VimbaCamera*>(frame->context[0]);

        cv::Mat wrapper(
            cv::Size(frame->width, frame->height),
            CV_8UC3,
            frame->buffer);

        camera->m_mutex.lock();

        camera->m_newest_image.setTimestamp( double(frame->timestamp) / double(camera->m_tick_frequency) );
        camera->m_newest_image.refFrame() = wrapper.clone();
        camera->m_newest_image.setValid(true);

        camera->m_mutex.unlock();
    }

    VmbCaptureFrameQueue( handle, frame, &VimbaCamera::frame_callback );
}
*/

VimbaCamera::VimbaCamera(const VmbCameraInfo_t& infos)
{
    m_camera_id = infos.cameraIdString;
    m_camera_name = infos.cameraName;
    m_camera_model = infos.modelName;
    m_camera_serial = infos.serialString;
    m_camera_permitted_access = infos.permittedAccess;
    m_interface_id = infos.interfaceIdString;
    m_is_open = false;
}

VimbaCamera::~VimbaCamera()
{
    if( m_is_open )
    {
        close();
    }
}

std::string VimbaCamera::getHumanName()
{
    //return m_camera_name;
    return m_camera_model;
}

bool VimbaCamera::open()
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
            err = VmbFeatureEnumSet(m_handle, "AcquisitionMode", "Continuous");
            ok = (VmbErrorSuccess == err);
        }

        if(ok)
        {
            err = VmbFeatureEnumSet(m_handle, "PixelFormat", "BGR8Packed");
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
            m_frames.resize(2);

            for(Frame& f : m_frames)
            {
                f.buffer.resize(payload_size);
                f.frame.buffer = &f.buffer.front();
                f.frame.bufferSize = payload_size;
                f.frame.context[0] = nullptr;
                f.frame.context[1] = nullptr;
                f.frame.context[2] = nullptr;
                f.frame.context[3] = nullptr;

                err = VmbFrameAnnounce(m_handle, &f.frame, sizeof(VmbFrame_t));
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
                err = VmbCaptureFrameQueue(m_handle, &f.frame, nullptr);
                ok = ok && (VmbErrorSuccess == err);
            }
        }

        if(ok)
        {
            err = VmbFeatureCommandRun(m_handle, "AcquisitionStart");
            ok = (VmbErrorSuccess == err);
        }
    }

    // for the time being, we make sure that an error will be remarked. TODO

    if(ok)
    {
        m_is_open = true;
    }
    else
    {
        throw std::runtime_error("Error while starting acquisition from the camera");
    }

    return ok;
}

void VimbaCamera::close()
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

void VimbaCamera::read(Image& image)
{
    image.setValid(false);

    if( m_is_open )
    {
        Frame& f = m_frames[m_next_frame];

        VmbError_t err = VmbCaptureFrameWait(m_handle, &f.frame, 10);

        if(err == VmbErrorSuccess)
        {
            m_next_frame = (m_next_frame+1) % m_frames.size();

            if(
                (f.frame.receiveStatus == VmbFrameStatusComplete) &&
                (f.frame.pixelFormat == VmbPixelFormatBgr8) &&
                (f.frame.receiveFlags & VmbFrameFlagsTimestamp) &&
                (f.frame.receiveFlags & VmbFrameFlagsDimension) )
            {
                cv::Mat wrapper(
                    cv::Size(f.frame.width, f.frame.height),
                    CV_8UC3,
                    f.frame.buffer);

                image.setTimestamp( double(f.frame.timestamp) / double(m_tick_frequency) );
                image.refFrame() = wrapper.clone();
                image.setValid(true);
            }
            /*
            else
            {
                std::cout << "Ahh!" << std::endl;
                std::cout << f.frame.receiveStatus << std::endl;
                std::cout << f.frame.pixelFormat << std::endl;
                std::cout << (f.frame.receiveFlags & VmbFrameFlagsTimestamp) << std::endl;
                std::cout << (f.frame.receiveFlags & VmbFrameFlagsDimension) << std::endl;
            }
            */

            VmbCaptureFrameQueue( m_handle, &f.frame, nullptr);
        }
    }
}

// definition of VimbaCameraManagerImpl

VimbaCameraManagerImpl VimbaCameraManagerImpl::m_instance;

VimbaCameraManager& VimbaCameraManager::instance()
{
    return VimbaCameraManagerImpl::instance();
}

bool VimbaCameraManagerImpl::initialize()
{
    bool ok = true;
    VmbError_t err = VmbErrorSuccess;
    VmbUint32_t count = 0;
    std::vector<VmbCameraInfo_t> info;

    if(ok)
    {
        ok = (VmbErrorSuccess == VmbStartup());
    }

    if(ok)
    {
        bool gige;
        ok =
            ( VmbFeatureBoolGet(gVimbaHandle, "GeVTLIsPresent", &gige) == VmbErrorSuccess && gige );
    }

    if(ok)
    {
        ok = (VmbErrorSuccess == VmbFeatureCommandRun(gVimbaHandle, "GeVDiscoveryAllOnce"));
    }

    // get number of cameras.

    if(ok)
    {
        ok = (VmbErrorSuccess == VmbCamerasList(NULL, 0, &count, sizeof(VmbCameraInfo_t)));
    }

    // retrieve camera infos.

    if( ok && count > 0 )
    {
        info.resize(count);
        ok = (VmbErrorSuccess == VmbCamerasList(&info.front(), count, &count, sizeof(VmbCameraInfo_t)));
    }

    // create cameras.

    if(ok && count > 0 )
    {
        m_cameras.resize(count);

        for(int i=0; i<count; i++)
        {
            m_cameras[i].reset( new VimbaCamera(info[i]) );
        }
    }

    if(ok == false)
    {
        m_cameras.clear();
        VmbShutdown();
    }

    return ok;
}

void VimbaCameraManagerImpl::finalize()
{
    for( CameraPtr& camera : m_cameras)
    {
        if(camera.use_count() != 1)
        {
            throw std::runtime_error("Attempted to release VimbaCameraManager while a VimbaCamera is still referenced.");
        }
    }

    m_cameras.clear();

    VmbShutdown();
}

CameraPtr VimbaCameraManagerImpl::getDefaultCamera()
{
    if( m_cameras.empty() )
    {
        return CameraPtr();
    }
    else
    {
        return m_cameras.front();
    }
}

int VimbaCameraManagerImpl::getNumCameras()
{
    return m_cameras.size();
}

CameraPtr VimbaCameraManagerImpl::getCamera(int id)
{
    if( 0 <= id && id < m_cameras.size() )
    {
        return m_cameras[id];
    }
    else
    {
        return CameraPtr();
    }
}


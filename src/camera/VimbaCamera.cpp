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

    static void VMB_CALL frame_callback( const VmbHandle_t camera, VmbFrame_t* frame );

protected:

    std::string m_camera_id;
    std::string m_camera_name;
    std::string m_camera_model;
    std::string m_camera_serial;
    VmbAccessMode_t m_camera_permitted_access;
    std::string m_interface_id;

    bool m_is_open;
    VmbHandle_t m_handle;
    VmbFrame_t m_frame;
    std::vector<uint8_t> m_buffer;
    std::mutex m_mutex;
    Image m_newest_image;
    VmbInt64_t m_tick_frequency;
};

// declaration of VimbaCameraManagerImpl

class VimbaCameraManagerImpl : public VimbaCameraManager
{

public:

    static VimbaCameraManagerImpl& instance() { return m_instance; }

    bool initialize() override;

    void finalize() override;

    std::shared_ptr<Camera> getDefaultCamera() override;

    int getNumCameras() override;

    std::shared_ptr<Camera> getCamera(int id) override;

protected:

    std::vector< std::shared_ptr<Camera> > m_cameras;
    static VimbaCameraManagerImpl m_instance;
};

// VimbaCamera

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
    /*
    else
    {
        std::cout << "error while receiving frame !" << std::endl;
    }
    */

    VmbCaptureFrameQueue( handle, frame, &VimbaCamera::frame_callback );
}

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
            m_buffer.resize(payload_size);
            m_frame.buffer = &m_buffer.front();
            m_frame.bufferSize = payload_size;
            m_frame.context[0] = this;
            m_frame.context[1] = nullptr;
            m_frame.context[2] = nullptr;
            m_frame.context[3] = nullptr;

            err = VmbFrameAnnounce(m_handle, &m_frame, sizeof(VmbFrame_t));
            ok = (VmbErrorSuccess == err);
        }

        if(ok)
        {
            err = VmbCaptureStart(m_handle);
            ok = (VmbErrorSuccess == err);
        }

        if(ok)
        {
            err = VmbCaptureFrameQueue(m_handle, &m_frame, &VimbaCamera::frame_callback);
            ok = (VmbErrorSuccess == err);
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

        m_buffer.clear();

        m_is_open = false;
    }
}

void VimbaCamera::read(Image& image)
{
    if( m_is_open )
    {
        m_mutex.lock();

        m_newest_image.moveTo(image);

        m_mutex.unlock();
    }
    else
    {
        image.setValid(false);
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
    for( std::shared_ptr<Camera>& camera : m_cameras)
    {
        if(camera.use_count() != 1)
        {
            throw std::runtime_error("Attempted to release VimbaCameraManager while a VimbaCamera is still referenced.");
        }
    }

    m_cameras.clear();

    VmbShutdown();
}

std::shared_ptr<Camera> VimbaCameraManagerImpl::getDefaultCamera()
{
    if( m_cameras.empty() )
    {
        return std::shared_ptr<Camera>();
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

std::shared_ptr<Camera> VimbaCameraManagerImpl::getCamera(int id)
{
    return m_cameras[id];
}


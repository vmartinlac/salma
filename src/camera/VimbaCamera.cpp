#include <cassert>
#include <iostream>
#include <stdexcept>
#include <opencv2/core.hpp>
#include "VimbaCamera.h"
#include "Image.h"

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

VimbaCamera::VimbaCamera(int id, const VmbCameraInfo_t& infos) : Camera(id)
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
            assert( m_frames.empty() );

            const int num_frames = 2;
            m_frames.resize(num_frames);

            for( VmbFrame_t& frame : m_frames )
            {
                frame.buffer = new uint8_t[payload_size];
                frame.bufferSize = payload_size;
                frame.context[0] = this;
                frame.context[1] = nullptr;
                frame.context[2] = nullptr;
                frame.context[3] = nullptr;

                err = VmbFrameAnnounce(m_handle, &frame, sizeof(VmbFrame_t));
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
            for(VmbFrame_t& frame : m_frames)
            {
                err = VmbCaptureFrameQueue(m_handle, &frame, &VimbaCamera::frame_callback);
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

        for(VmbFrame_t& frame : m_frames)
        {
            delete[] ( (uint8_t*) frame.buffer );
        }

        m_frames.clear();

        m_is_open = false;
    }
}

bool VimbaCamera::read(Image& image)
{
    if( m_is_open )
    {
        m_mutex.lock();

        m_newest_image.moveTo(image);

        m_mutex.unlock();

        return true;
    }
    else
    {
        return false;
    }
}

// CameraManager

bool VimbaCameraManager::initialize()
{
    bool ok = true;
    VmbError_t err = VmbErrorSuccess;
    VmbUint32_t count = 0;
    VmbCameraInfo_t* info = nullptr;

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
        ok = (VmbErrorSuccess == VmbCamerasList(NULL, 0, &count, sizeof(*info)));
    }

    // retrieve camera infos.

    if( ok && count > 0 )
    {
        info = new VmbCameraInfo_t[count];
        ok = (VmbErrorSuccess == VmbCamerasList(info, count, &count, sizeof(*info)));
    }

    // create cameras.

    if(ok && count > 0 )
    {
        m_cameras.resize(count);

        for(int i=0; i<count; i++)
        {
            m_cameras[i] = new VimbaCamera(i, info[i]);
        }
    }

    if(info != nullptr)
    {
        delete[] info;
    }

    if(ok == false)
    {
        m_cameras.clear();
        VmbShutdown();
    }

    return ok;
}

void VimbaCameraManager::finalize()
{
    for(Camera* camera : m_cameras)
    {
        delete camera;
    }

    VmbShutdown();
}

Camera* VimbaCameraManager::getDefaultCamera()
{
    if( m_cameras.empty() )
    {
        return nullptr;
    }
    else
    {
        return m_cameras.front();
    }
}

int VimbaCameraManager::getNumCameras()
{
    return m_num_cameras;
}

Camera* VimbaCameraManager::getCamera(int id)
{
    return m_cameras[id];
}

CameraManager* CameraManager::createVimbaCameraManager()
{
    return new VimbaCameraManager();
}

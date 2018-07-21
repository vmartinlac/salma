#include <cassert>
#include <iostream>
#include <stdexcept>
#include "VimbaCamera.h"
#include "Image.h"

// VimbaCamera

void VMB_CALL VimbaCamera::frame_callback(
    const VmbHandle_t handle,
    VmbFrame_t* frame)
{
    VimbaCamera* camera = static_cast<VimbaCamera*>(frame->context[0]);

    if( VmbFrameStatusComplete == frame->receiveStatus )
    {
        Image* new_image = new Image();

        new_image->setTimestamp(frame->timestamp);

        cv::Mat& frame = new_image->frame();
        
        frame.create(640, 480, CV_32FC3);
        frame = cv::Scalar(0.0f, 0.0f, 0.0f);

        // TODO : fill the frame.

        camera->m_mutex.lock();

        if( camera->m_newest_image != nullptr )
        {
            delete camera->m_newest_image;
        }
        camera->m_newest_image = new_image;

        camera->m_mutex.unlock();
    }
    else
    {
        std::cout << "error while receiving frame !" << std::endl;
    }

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
    m_newest_image = nullptr;
}

VimbaCamera::~VimbaCamera()
{
    if( m_is_open )
    {
        stop();
    }
}

std::string VimbaCamera::getHumanName()
{
    return m_camera_name;
}

bool VimbaCamera::start()
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
            err = VmbFeatureIntGet(m_handle, "PayloadSize", &payload_size);
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

    // for the time being, we make sure that an error will be remarked.

    if(ok == false)
    {
        throw std::runtime_error("Error while starting acquisition from the camera");
    }

    return ok;
}

void VimbaCamera::stop()
{
    if( m_is_open )
    {
        bool ok = true;

        ok = ok && ( VmbErrorSuccess == VmbFeatureCommandRun(m_handle, "AcquisitionStop") );

        ok = ok && ( VmbErrorSuccess == VmbCaptureEnd( m_handle ) );

        ok = ok && (VmbErrorSuccess == VmbCaptureQueueFlush( m_handle ) );

        ok = ok && (VmbErrorSuccess == VmbFrameRevokeAll( m_handle ) );

        ok = ok && (VmbErrorSuccess == VmbCameraClose( m_handle ) );

        for(VmbFrame_t& frame : m_frames)
        {
            delete[] ( (uint8_t*) frame.buffer );
        }

        m_frames.clear();

        if( m_newest_image != nullptr )
        {
            delete m_newest_image;
        }

        m_is_open = false;
    }
}

Image* VimbaCamera::readImage()
{
    Image* ret = nullptr;

    if( m_is_open )
    {
        m_mutex.lock();

        ret = m_newest_image;
        m_newest_image = nullptr;

        m_mutex.unlock();
    }

    return ret;
}

// CameraManager

bool VimbaCameraManager::initialize()
{
    bool ok = true;
    VmbError_t err = VmbErrorSuccess;
    VmbUint32_t count = 0;
    VmbCameraInfo_t* info = nullptr;

    VmbStartup();

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
    return m_cameras[0];
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

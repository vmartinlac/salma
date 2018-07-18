#include "VimbaCamera.h"

// VimbaCamera

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
    bool ok = true;

    if( m_is_open == false )
    {
        VmbError_t err;
        
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
            // TO BE CONTINUED !
        }

        if(ok)
        {
            VmbCameraClose(m_handle);
            m_is_open = true;
        }
    }

    return ok;
}

void VimbaCamera::stop()
{
    if( m_is_open )
    {
        VmbCameraClose( m_handle );
        m_is_open = false;
    }
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

CameraManager* CameraManager::createDefaultCameraManager()
{
    return new VimbaCameraManager();
}

#include <chrono>
#include "OpenCVCamera.h"
#include "Image.h"

OpenCVCameraManager::OpenCVCameraManager() : m_camera(0)
{
    ;
}

OpenCVCamera::OpenCVCamera(int id) : Camera(id)
{
    ;
}

OpenCVCamera::~OpenCVCamera()
{
    close();
}

std::string OpenCVCamera::getHumanName()
{
    return "OpenCV camera " + std::to_string(getId());
}

bool OpenCVCamera::open()
{
    if( m_video.isOpened() )
    {
        return false;
    }
    else
    {
        m_t0 = std::chrono::high_resolution_clock::now();
        m_video.open(getId());
        return m_video.isOpened();
    }
}

void OpenCVCamera::close()
{
    if( m_video.isOpened() )
    {
        m_video.release();
    }
}

bool OpenCVCamera::read(Image& image)
{
    m_video.grab();
    m_video >> image.refFrame();

    const auto elapsed_time = std::chrono::duration_cast<std::chrono::seconds>( std::chrono::high_resolution_clock::now() - m_t0 ).count();
    image.setTimestamp( elapsed_time );
    image.setValid(true);

    return true;
}


bool OpenCVCameraManager::initialize()
{
    return true;
}

void OpenCVCameraManager::finalize()
{
    ;
}

Camera* OpenCVCameraManager::getDefaultCamera()
{
    return &m_camera;
}

int OpenCVCameraManager::getNumCameras()
{
    return 1;
}

Camera* OpenCVCameraManager::getCamera(int id)
{
    if( id == 0 )
    {
        return &m_camera;
    }
    else
    {
        return nullptr;
    }
}

CameraManager* CameraManager::createOpenCVCameraManager()
{
    return new OpenCVCameraManager();
}

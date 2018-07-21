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
    stop();
}

std::string OpenCVCamera::getHumanName()
{
    return "OpenCV camera " + std::to_string(getId());
}

bool OpenCVCamera::start()
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

void OpenCVCamera::stop()
{
    if( m_video.isOpened() )
    {
        m_video.release();
    }
}

Image* OpenCVCamera::readImage()
{
    Image* ret = new Image();

    m_video.grab();
    m_video >> ret->frame();

    const auto elapsed_time = std::chrono::duration_cast<std::chrono::seconds>( std::chrono::high_resolution_clock::now() - m_t0 ).count();
    ret->setTimestamp( elapsed_time );

    return ret;
}


bool OpenCVCameraManager::initialize()
{
    ;
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

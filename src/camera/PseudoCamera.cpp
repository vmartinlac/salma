#include <opencv2/imgcodecs.hpp>
#include <cstdlib>
#include "PseudoCamera.h"

PseudoCamera::PseudoCamera() : Camera(0)
{
    bool ok = false;

    char* name = getenv("SLAM_IMAGE");
    if(name != nullptr)
    {
        m_image = cv::imread(name);
        ok = (m_image.data != nullptr);
    }

    if( ok == false )
    {
        m_image.create(1024, 768, CV_8UC3);
        m_image = cv::Scalar(64,128,64);
    }
}

PseudoCamera::~PseudoCamera()
{
}

std::string PseudoCamera::getHumanName()
{
    return "Pseudo-camera";
}

bool PseudoCamera::open()
{
    m_t0 = std::chrono::steady_clock::now();
    m_t1 = m_t0;
    return true;
}

void PseudoCamera::close()
{
}

bool PseudoCamera::read(Image& image)
{
    std::chrono::time_point< std::chrono::steady_clock > t2 = std::chrono::steady_clock::now();

    const double dt = static_cast<double>( std::chrono::duration_cast<std::chrono::milliseconds>(t2-m_t1).count() )*1.0e-3;

    if( dt > 1.0/30.0 )
    {
        const double time = static_cast<double>( std::chrono::duration_cast<std::chrono::milliseconds>(t2-m_t0).count() )*1.0e-3;
        m_t1 = t2;

        image.refFrame() = m_image.clone();
        image.setTimestamp(time);
        image.setValid(true);
    }
    else
    {
        image.setValid(false);
    }

    return true;
}

bool PseudoCameraManager::initialize()
{
    return true;
}

void PseudoCameraManager::finalize()
{
}

Camera* PseudoCameraManager::getDefaultCamera()
{
    return &m_camera;
}

int PseudoCameraManager::getNumCameras()
{
    return 1;
}

Camera* PseudoCameraManager::getCamera(int id)
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

CameraManager* CameraManager::createPseudoCameraManager()
{
    return new PseudoCameraManager();
}


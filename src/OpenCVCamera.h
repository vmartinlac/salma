
#pragma once

#include <opencv2/videoio.hpp>
#include <chrono>
#include <string>
#include "Camera.h"

class OpenCVCamera : public Camera
{

public:

    OpenCVCamera(int id);
    ~OpenCVCamera() override;

    std::string getHumanName() override;

    bool start() override;

    void stop() override;

    Image* readImage() override;

protected:

    cv::VideoCapture m_video;
    std::chrono::time_point<std::chrono::high_resolution_clock> m_t0;
};

class OpenCVCameraManager : public CameraManager
{

public:

    OpenCVCameraManager();

    bool initialize() override;

    void finalize() override;

    Camera* getDefaultCamera() override;

    int getNumCameras() override;

    Camera* getCamera(int id) override;

protected:

    OpenCVCamera m_camera;
};


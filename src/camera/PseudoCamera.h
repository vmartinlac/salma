
#pragma once

#include <chrono>
#include <string>
#include <vector>
#include <opencv2/core.hpp>
#include "Camera.h"
#include "Image.h"

// PseudoCamera

class PseudoCamera : public Camera
{

public:

    PseudoCamera();
    ~PseudoCamera() override;

    std::string getHumanName() override;

    bool open() override;

    void close() override;

    bool read(Image& image) override;

protected:

    cv::Mat m_image;
    std::chrono::time_point< std::chrono::steady_clock > m_t0;
    std::chrono::time_point< std::chrono::steady_clock > m_t1;
};

// CameraManager

class PseudoCameraManager : public CameraManager
{

public:

    bool initialize() override;

    void finalize() override;

    Camera* getDefaultCamera() override;

    int getNumCameras() override;

    Camera* getCamera(int id) override;

protected:

   PseudoCamera m_camera;
};


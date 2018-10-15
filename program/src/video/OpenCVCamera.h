
#pragma once

#include <opencv2/videoio.hpp>
#include <string>
#include <chrono>
#include "Image.h"
#include "Camera.h"

class OpenCVCamera : public Camera
{

public:

    OpenCVCamera(int id);

    ~OpenCVCamera() override;

    std::string getHumanName() override;

    bool open() override;

    void close() override;

    void read(Image& image) override;

protected:

    cv::VideoCapture m_video;
    int m_id;
    double m_t0;
    bool m_first;
};

class OpenCVVideoFile : public Camera
{

public:

    OpenCVVideoFile(const std::string& filename);

    ~OpenCVVideoFile() override;

    std::string getHumanName() override;

    bool open() override;

    void close() override;

    void read(Image& image) override;

protected:

    std::string m_filename;
    cv::VideoCapture m_video;
    bool m_first;
    std::chrono::time_point< std::chrono::steady_clock > m_t0;
    Image m_next_image;
};


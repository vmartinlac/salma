#pragma once

#include <opencv2/core.hpp>
#include <Eigen/Eigen>
#include "SLAMEngine.h"

class DefaultSLAMEngine : public SLAMEngine
{
public:

    DefaultSLAMEngine();

    ~DefaultSLAMEngine() override;

    void initialize() override;

    void processNextView(Image* image) override;

    std::string name() override;

protected:

    Image* m_prev_image;
    cv::Mat m_prev_pose;
};


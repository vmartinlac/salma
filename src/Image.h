#pragma once

#include <opencv2/core.hpp>

class Image
{
public:
    int width();
    int height();
    double timestamp();
    cv::Mat& frame();
protected:
    cv::Mat m_mat;
};

#pragma once

#include <opencv2/core.hpp>

class Image
{

public:

    int width();

    int height();

    double timestamp();

    void setTimestamp(double t);

    cv::Mat& frame();

protected:

    cv::Mat m_mat;

    double m_timestamp;
};

inline void Image::setTimestamp(double t)
{
    m_timestamp = t;
}

inline int Image::width() 
{
    return m_mat.cols;
}

inline int Image::height() 
{
    return m_mat.rows;
}

inline cv::Mat& Image::frame() 
{
    return m_mat;
}

inline double Image::timestamp()
{
    return m_timestamp;
}


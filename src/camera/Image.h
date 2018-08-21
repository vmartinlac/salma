#pragma once

#include <opencv2/core.hpp>

class Image
{

public:

    Image();

    Image(const Image&) = delete;
    Image& operator=(const Image&) = delete;

    void moveTo(Image& other);

    bool isValid();
    void setValid(bool v);

    double getTimestamp();
    void setTimestamp(double t);

    int width();
    int height();

    cv::Mat& refFrame();

protected:

    bool m_valid;
    double m_timestamp;
    cv::Mat m_mat;
};


inline void Image::moveTo(Image& o)
{
    o.m_valid = m_valid;
    o.m_timestamp = m_timestamp;
    o.m_mat = std::move(m_mat);

    m_valid = false;
    m_timestamp = 0;
    m_mat.release();
}

inline Image::Image()
{
    m_valid = false;
    m_timestamp = 0;
}

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

inline cv::Mat& Image::refFrame() 
{
    return m_mat;
}

inline double Image::getTimestamp()
{
    return m_timestamp;
}

inline void Image::setValid(bool v)
{
    m_valid = v;
}

inline bool Image::isValid()
{
    return m_valid;
}


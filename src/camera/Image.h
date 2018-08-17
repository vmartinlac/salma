#pragma once

#include <opencv2/core.hpp>

class Image
{

public:

    Image();
    Image(const Image& o) = default;
    Image(Image&& o);

    Image& operator=(const Image& o) = default;
    void operator=(Image&& o);

    bool isValid() { return m_valid; }
    void setValid(bool v) { m_valid = v; }

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

inline Image::Image(Image&& o)
{
    operator=(std::move(o)); // TODO: move or forward ?
}

inline void Image::operator=(Image&& o)
{
    m_valid = o.m_valid;
    m_timestamp = o.m_timestamp;
    m_mat = std::move(o.m_mat);

    o.m_valid = false;
    o.m_timestamp = 0;
    o.m_mat.release();
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


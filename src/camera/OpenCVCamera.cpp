#include <chrono>
#include <iostream>
#include "OpenCVCamera.h"

OpenCVCamera::OpenCVCamera(int id)
{
    m_id = id;
    m_first = true;
    m_t0 = 0.0;
}

OpenCVCamera::~OpenCVCamera()
{
    close();
}

std::string OpenCVCamera::getHumanName()
{
    return "OpenCV camera " + std::to_string(m_id);
}

bool OpenCVCamera::open()
{
    if(m_video.isOpened())
    {
        return false;
    }
    else
    {
        m_first = true;
        m_t0 = 0.0;
        m_video.open(m_id);
        return m_video.isOpened();
    }
}

void OpenCVCamera::close()
{
    if( m_video.isOpened() )
    {
        m_video.release();
    }
}

void OpenCVCamera::read(Image& image)
{
    if( m_video.isOpened() )
    {
        const bool ret = m_video.read( image.refFrame() );

        if(ret)
        {
            const double t1 = m_video.get(cv::CAP_PROP_POS_MSEC)*1.0e-3;

            if( m_first )
            {
                m_first = false;
                m_t0 = t1;
            }

            image.setTimestamp( t1 - m_t0 );

            image.setValid(true);
        }
        else
        {
            image.setValid(false);
        }
    }
    else
    {
        image.setValid(false);
    }
}

OpenCVVideoFile::OpenCVVideoFile(const std::string& filename)
{
    m_filename = filename;
}

OpenCVVideoFile::~OpenCVVideoFile()
{
    close();
}

std::string OpenCVVideoFile::getHumanName()
{
    return "OpenCV file " + m_filename;
}

bool OpenCVVideoFile::open()
{
    if(m_video.isOpened())
    {
        return false;
    }
    else
    {
        m_first = true;
        m_video.open(m_filename);
        return m_video.isOpened();
    }
}

void OpenCVVideoFile::close()
{
    if( m_video.isOpened() )
    {
        m_video.release();
    }
}

void OpenCVVideoFile::read(Image& image)
{
    if( m_video.isOpened() )
    {
        if(m_first)
        {
            const bool ret = m_video.read( m_next_image.refFrame() );

            if(ret)
            {
                m_next_image.setTimestamp( m_video.get(cv::CAP_PROP_POS_MSEC)*1.0e-3 );
                m_next_image.setValid(true);

                m_t0 = std::chrono::steady_clock::now();

                m_first = false;
            }
            else
            {
                m_next_image.setValid(false);
            }
        }
        else if(m_next_image.isValid())
        {
            std::chrono::time_point< std::chrono::steady_clock > t1 = std::chrono::steady_clock::now();

            const double delta_t = double(std::chrono::duration_cast<std::chrono::milliseconds>(t1-m_t0).count()) * 1.0e-3;

            if( delta_t > m_next_image.getTimestamp() )
            {
                m_next_image.moveTo(image);

                const bool ret = m_video.read( m_next_image.refFrame() );

                if(ret)
                {
                    m_next_image.setTimestamp( m_video.get(cv::CAP_PROP_POS_MSEC)*1.0e-3 );
                    m_next_image.setValid(true);
                }
                else
                {
                    m_next_image.setValid(false);
                }
            }
            else
            {
                image.setValid(false);
            }
        }
        else
        {
            image.setValid(false);
        }
    }
    else
    {
        image.setValid(false);
    }
}


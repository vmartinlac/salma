#include <opencv2/imgproc.hpp>
#include "MockCamera.h"

MockCamera::MockCamera(int width, int height)
{
    mWidth = width;
    mHeight = height;
    mReady = false;
}

MockCamera::~MockCamera()
{
}


bool MockCamera::open()
{
    mT0 = std::chrono::steady_clock::now();
    mReady = true;
    return true;
}

void MockCamera::close()
{
    mReady = false;
}


std::string MockCamera::getHumanName()
{
    return "Mock camera";
}


void MockCamera::read(Image& image)
{
    if(mReady)
    {
        cv::Mat frame( cv::Size(mWidth, mHeight), CV_8UC3);
        frame = cv::Scalar(0, 0, 0);
        cv::circle(frame, cv::Point2f(mWidth/2, mHeight/2), mWidth/4, cv::Scalar(0, 255, 0), -1);

        int ms = std::chrono::duration_cast<std::chrono::milliseconds>( std::chrono::steady_clock::now() - mT0 ).count();
        const double timestamp = double(ms) * 1.0e-3;

        image.setValid(timestamp, frame);
    }
}

void MockCamera::trigger()
{
}


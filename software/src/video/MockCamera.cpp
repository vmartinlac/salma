#include <opencv2/imgproc.hpp>
#include "MockCamera.h"

MockCamera::MockCamera(int num_views, int width, int height)
{
    mWidth = width;
    mHeight = height;
    mNumViews = num_views;
    mReady = false;
}

MockCamera::~MockCamera()
{
}

int MockCamera::getNumberOfCameras()
{
    return mNumViews;
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
        // compute timestamp.

        int ms = std::chrono::duration_cast<std::chrono::milliseconds>( std::chrono::steady_clock::now() - mT0 ).count();
        const double timestamp = double(ms) * 1.0e-3;

        // compute frames.

        std::vector<cv::Mat> frames;
        for(int i=0; i<mNumViews; i++)
        {
            cv::Mat frame( cv::Size(mWidth, mHeight), CV_8UC3);
            frame = cv::Scalar(0, 0, 0);
            cv::circle(frame, cv::Point2f(mWidth/2, mHeight/2), mWidth/4, cv::Scalar(0, 255, 0), -1);

            frames.push_back(frame);
        }

        if(mNumViews != frames.size()) throw std::logic_error("internal error");

        image.setValid(timestamp, frames);
    }
    else
    {
        image.setInvalid();
    }
}

void MockCamera::trigger()
{
}


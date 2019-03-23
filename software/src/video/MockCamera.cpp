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

        const int ms = std::chrono::duration_cast<std::chrono::milliseconds>( std::chrono::steady_clock::now() - mT0 ).count();
        const double timestamp = static_cast<double>(ms) * 1.0e-3;

        // compute frames.

        std::vector<cv::Mat> frames;
        for(int i=0; i<mNumViews; i++)
        {
            const double kappa = 0.5 * (1.0 + std::cos(M_PI*timestamp/1.0));

            const double radius = static_cast<double>(mWidth/4) * kappa + 10.0 * (1.0-kappa);
            
            const cv::Scalar color(0, 255, 0);

            cv::Point2f center(mWidth/2, mHeight/2);

            cv::Mat frame( cv::Size(mWidth, mHeight), CV_8UC3 );

            frame = cv::Vec3b(0, 0, 0);

            cv::circle(frame, center, radius, color, -1);

            const std::string text = "View #" + std::to_string(i);

            int baseline = 0;
            cv::Size textsize = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 1.0, 1, &baseline);

            cv::putText(
                frame,
                text,
                cv::Point2i(1,1+textsize.height),
                cv::FONT_HERSHEY_SIMPLEX,
                1.0,
                cv::Scalar(0.0, 255.0, 0.0),
                1,
                cv::LINE_8,
                false);

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


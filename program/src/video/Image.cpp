#include "Image.h"

Image::Image()
{
    mValid = false;
}

void Image::setInvalid()
{
    mValid = false;
}

void Image::setValid(double timestamp, const cv::Mat& frame)
{
    mValid = true;
    mTimestamp = timestamp;
    mNumberOfFrames = 1;
    mFrames.at(0) = frame;
}

void Image::setValid(double timestamp, const cv::Mat& left_frame, const cv::Mat& right_frame)
{
    mValid = true;
    mTimestamp = timestamp;
    mNumberOfFrames = 2;
    mFrames.at(0) = left_frame;
    mFrames.at(1) = right_frame;
}

void Image::setValid(double timestamp, const std::vector<cv::Mat>& frames)
{
    if( frames.size() > mFrames.size() ) throw std::runtime_error("Too many frames!");
    mValid = true;
    mTimestamp = timestamp;
    mNumberOfFrames = frames.size();
    std::copy(frames.begin(), frames.end(), mFrames.begin());
}

bool Image::isValid()
{
    return mValid;
}

int Image::getNumberOfFrames()
{
    if( mValid )
    {
        return mNumberOfFrames;
    }
    else
    {
        return 0;
    }
}

double Image::getTimestamp()
{
    if( mValid )
    {
        return mTimestamp;
    }
    else
    {
        return 0.0;
    }
}

cv::Mat& Image::getFrame(int idx)
{
    if( mValid == false || idx < 0 || idx >= mNumberOfFrames )
    {
        throw std::runtime_error("Invalid image or incorrect frame number");
    }
    else
    {
        return mFrames.at(idx);
    }
}


#pragma once

#include <opencv2/core.hpp>

class StereoImage
{
public:

    StereoImage();

    bool isValid();
    void setValid(bool s);

    double getTimestamp();

    cv::Mat& getLeftImage();
    cv::Mat& getRightImage();

    double getLeftTimestamp();
    double getRightTimestamp();

protected:

    bool mValid;

    double mTimestamp;

    cv::Mat mLeftImage;
    double mLeftTimestamp;

    cv::Mat mRightImage;
    double mRightTimestamp;
};

inline StereoImage::StereoImage()
{
    mValid = false;
    mTimestamp = 0.0;
    mLeftTimestamp = 0.0;
    mRightTimestamp = 0.0;
}

inline bool StereoImage::isValid()
{
    return mValid;
}

inline void StereoImage::setValid(bool val)
{
    mValid = val;
}


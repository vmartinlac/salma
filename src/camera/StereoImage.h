#pragma once

#include <opencv2/core.hpp>

class StereoImage
{
public:

    bool isValid();

    double getTimestamp();

    cv::Mat& getLeftImage();
    cv::Mat& getRightImage();

    double getLeftTimestamp();
    double getRightTimestamp();

protected:

    bool mValid;
    double mTimestamp;
    cv::Mat mLeftImage;
    cv::Mat mRightImage;
    double mLeftTimestamp;
    double mRightTimestamp;
};


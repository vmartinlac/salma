
#pragma once

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <utility>
#include <cmath>
#include "SLAMModule.h"

class SLAMModuleFeatures : public SLAMModule
{
public:

    SLAMModuleFeatures(SLAMProjectPtr project);

    void run(cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors);

protected:

    struct Level
    {
        cv::Mat image;
        double scale;
    };

protected:

    void buildPyramid(cv::Mat& image);
    void detectAllKeyPoints();
    void binKeyPoints(bool second);
    void describeKeyPoints();

protected:

    std::vector<Level> mPyramid;
    std::vector<cv::KeyPoint> mKeyPoints;
    cv::Mat mDescriptors;
};

typedef std::shared_ptr<SLAMModuleFeatures> SLAMModuleFeaturesPtr;


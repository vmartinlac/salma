
#pragma once

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <utility>
#include <cmath>
#include "SLAMDataStructures.h"
#include "SLAMModule.h"

class SLAMModuleFeatures : public SLAMModule
{
public:

    SLAMModuleFeatures(SLAMContextPtr con);
    ~SLAMModuleFeatures() override;

    bool init() override;
    void operator()() override;

protected:

    void runOnView(cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors);

protected:

    cv::Ptr<cv::ORB> mFeature2d;
};


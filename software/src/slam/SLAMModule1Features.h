
#pragma once

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <utility>
#include <cmath>
#include "SLAMDataStructures.h"
#include "SLAMModule.h"

class SLAMModule1Features : public SLAMModule
{
public:

    SLAMModule1Features(SLAMContextPtr con);
    ~SLAMModule1Features() override;

    bool init() override;
    SLAMModuleResult operator()() override;

protected:

    void processView(SLAMView& v);
    void uniformize(std::vector<cv::KeyPoint>& keypoints);

protected:

    int mNumLevels;
    double mScaleFactor;
    cv::Ptr<cv::ORB> mFeature2d;
};



#pragma once

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <utility>
#include <cmath>
#include "SLAMDataStructures.h"
#include "SLAMModule.h"

class SLAMModule1Rectification : public SLAMModule
{
public:

    SLAMModule1Rectification(SLAMContextPtr con);
    ~SLAMModule1Rectification() override;

    bool init() override;
    SLAMModuleResult operator()() override;

protected:

    struct RectifiedCamera
    {
        cv::Mat P;
        cv::Mat R;
        cv::Mat map0;
        cv::Mat map1;
    };

    struct RectificationParameters
    {
        cv::Mat R;
        cv::Mat T;
        cv::Mat Q;
        RectifiedCamera cameras[2];
    };

protected:

    StereoRigCalibrationPtr mStereoRig;
    RectificationParameters mRectification;
};


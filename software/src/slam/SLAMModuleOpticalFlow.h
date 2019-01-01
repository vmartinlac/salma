
#pragma once

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video.hpp>
#include <utility>
#include <cmath>
#include "SLAMDataStructures.h"
#include "SLAMModule.h"

class SLAMModuleOpticalFlow : public SLAMModule
{
public:

    SLAMModuleOpticalFlow(SLAMContextPtr con);
    ~SLAMModuleOpticalFlow() override;

    void operator()() override;

protected:

    void processView(const SLAMView& view_prev, SLAMView& curr);

protected:

    cv::Ptr<cv::SparsePyrLKOpticalFlow> mLKT;
};


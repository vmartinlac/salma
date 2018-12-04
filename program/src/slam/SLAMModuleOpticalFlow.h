
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

    SLAMModuleOpticalFlow(SLAMProjectPtr project);

    void run(FramePtr frame);

protected:

    void processView(const View& view_prev, View& curr);

protected:

    cv::Ptr<cv::SparsePyrLKOpticalFlow> mLKT;
};

typedef std::shared_ptr<SLAMModuleOpticalFlow> SLAMModuleOpticalFlowPtr;


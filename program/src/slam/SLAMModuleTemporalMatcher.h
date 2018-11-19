#pragma once

#include <opencv2/video.hpp>
#include <memory>
#include "SLAMDataStructures.h"
#include "SLAMModule.h"

class SLAMModuleTemporalMatcher : public SLAMModule
{
public:

    SLAMModuleTemporalMatcher(SLAMProjectPtr project);

    void match(FramePtr frame);

protected:

    void processView(FramePtr frame, int view);

protected:

    cv::Ptr<cv::SparsePyrLKOpticalFlow> mLKT;

    bool mCheckSymmetry;

    bool mCheckLowe;
    double mLoweRatio;

    bool mCheckOpticalFlow;
    double mOpticalFlowRadius;

    bool mCheckOctave;
};

typedef std::shared_ptr<SLAMModuleTemporalMatcher> SLAMModuleTemporalMatcherPtr;


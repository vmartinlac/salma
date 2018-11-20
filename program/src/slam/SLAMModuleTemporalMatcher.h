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
    int matchKeyPoint(int i, const cv::Point2f& prediction, const View& from, const View& to, bool check_symmetry);

protected:

    cv::Ptr<cv::SparsePyrLKOpticalFlow> mLKT;

    bool mCheckSymmetry;

    bool mCheckLowe;
    double mLoweRatio;

    bool mCheckOctave;

    double mPredictionRadius;
};

typedef std::shared_ptr<SLAMModuleTemporalMatcher> SLAMModuleTemporalMatcherPtr;


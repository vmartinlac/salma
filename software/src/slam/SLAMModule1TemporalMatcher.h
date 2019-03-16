#pragma once

//#include <opencv2/video.hpp>
#include <set>
#include <memory>
#include "SLAMDataStructures.h"
#include "SLAMModule.h"

class SLAMModule1TemporalMatcher : public SLAMModule
{
public:

    SLAMModule1TemporalMatcher(SLAMContextPtr con);

    bool init() override;
    SLAMModuleResult operator()() override;

protected:

    void processView(
        SLAMFramePtr prev_frame,
        SLAMFramePtr curr_frame,
        int view,
        int& projection_count);

    int matchKeyPoint(int i, const SLAMView& from, const SLAMView& to, bool check_symmetry);

protected:

    bool mCheckSymmetry;
    bool mCheckLowe;
    double mLoweRatio;
    bool mCheckOctave;
    int mMaxDescriptorDistance;
    int mNumPreviousFrames;
    //int mMaxProjectedMapPointsPerView;

};

typedef std::shared_ptr<SLAMModule1TemporalMatcher> SLAMModule1TemporalMatcherPtr;


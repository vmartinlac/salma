#pragma once

//#include <opencv2/video.hpp>
#include <memory>
#include "SLAMDataStructures.h"
#include "SLAMModule.h"

class SLAMModuleTemporalMatcher : public SLAMModule
{
public:

    SLAMModuleTemporalMatcher(SLAMContextPtr con);

    bool init() override;
    void operator()() override;

protected:

    void processView(SLAMFramePtr prev_frame, SLAMFramePtr curr_frame, int view);
    int matchKeyPoint(int i, const SLAMView& from, const SLAMView& to, bool check_symmetry);

protected:

    bool mCheckSymmetry;

    bool mCheckLowe;
    double mLoweRatio;

    bool mCheckOctave;
};

typedef std::shared_ptr<SLAMModuleTemporalMatcher> SLAMModuleTemporalMatcherPtr;


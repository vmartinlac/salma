#pragma once

#include "SLAMContext.h"

enum SLAMModuleId
{
    SLAM_MODULE1_RECTIFICATION,
    SLAM_MODULE1_FEATURES,
    SLAM_MODULE1_TEMPORALMATCHER,
    SLAM_MODULE1_ALIGNMENT,
    SLAM_MODULE1_KEYFRAMESELECTION,
    SLAM_MODULE1_LOCALBUNDLEADJUSTMENT,
    SLAM_MODULE1_STEREOMATCHER,
    SLAM_MODULE1_TRIANGULATION,
    SLAM_MODULE1_DENSERECONSTRUCTION,

    SLAM_MODULE2_OPTICALFLOW,
    SLAM_MODULE2_EKF
};

class SLAMModuleResult
{
public:

    SLAMModuleResult(bool endpipeline, SLAMModuleId nextmodule)
    {
        mEndPipeline = endpipeline;
        mNextModule = nextmodule;
    }

    bool getEndPipeline()
    {
        return mEndPipeline;
    }

    SLAMModuleId getNextModule()
    {
        return mNextModule;
    }

protected:

    bool mEndPipeline;
    SLAMModuleId mNextModule;
};

class SLAMModule
{
public:

    SLAMModule(SLAMModuleId id, SLAMContextPtr con);
    virtual ~SLAMModule();

    virtual bool init();

    virtual SLAMModuleResult operator()() = 0;

    SLAMContextPtr context() { return mContext; }

    SLAMModuleId id() { return mId; }

private:

    SLAMContextPtr mContext;
    SLAMModuleId mId;
};

typedef std::shared_ptr<SLAMModule> SLAMModulePtr;


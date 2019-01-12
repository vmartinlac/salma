
#pragma once

//#include <opencv2/cudastereo.hpp>
#include <opencv2/core.hpp>
#include "SLAMDataStructures.h"
#include "SLAMModule.h"

class SLAMModuleDenseReconstruction : public SLAMModule
{
public:

    SLAMModuleDenseReconstruction(SLAMContextPtr con);
    ~SLAMModuleDenseReconstruction();

    bool init() override;

    void operator()() override;

protected:

    /*
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
    */

protected:

    /*
    CameraCalibrationDataPtr mCameras[2];
    StereoRigCalibrationDataPtr mStereoRig;

    RectificationParameters mRectification;
    */

    //cv::Ptr<cv::cuda::StereoBeliefPropagation> mSBP;
};


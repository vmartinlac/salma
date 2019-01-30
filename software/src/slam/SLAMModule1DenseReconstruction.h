
#pragma once

//#include <opencv2/cudastereo.hpp>
#include <opencv2/core.hpp>
#include "SLAMDataStructures.h"
#include "SLAMModule.h"

class SLAMModule1DenseReconstruction : public SLAMModule
{
public:

    SLAMModule1DenseReconstruction(SLAMContextPtr con);
    ~SLAMModule1DenseReconstruction();

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

    CameraCalibrationDataPtr mCameras[2];
    StereoRigCalibrationDataPtr mStereoRig;

    RectificationParameters mRectification;
};


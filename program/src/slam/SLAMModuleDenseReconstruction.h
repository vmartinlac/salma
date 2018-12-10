
#pragma once

#include <opencv2/core.hpp>
#include "SLAMDataStructures.h"
#include "SLAMModule.h"

class SLAMModuleDenseReconstruction : public SLAMModule
{
public:

    SLAMModuleDenseReconstruction(SLAMProjectPtr project);

    void run(FrameList& frames) override;

protected:

    void init();

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

typedef std::shared_ptr<SLAMModuleDenseReconstruction> SLAMModuleDenseReconstructionPtr;


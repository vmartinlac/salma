
#pragma once

#include "SLAMDataStructures.h"
#include "SLAMModule.h"

class SLAMModuleTriangulation : public SLAMModule
{
public:

    SLAMModuleTriangulation(SLAMProjectPtr project);

    void run(FramePtr frame);

protected:

    MapPointPtr triangulate(FramePtr frame, int left_keypoint, int right_keypoint);

protected:

    CameraCalibrationDataPtr mLeftCamera;
    CameraCalibrationDataPtr mRightCamera;
    StereoRigCalibrationDataPtr mRig;
    double mMinAngleBetweenRays;
    double mPerpendicularMaxLength;
    bool mCheckPerpendicularLength;
    int mInitialLifeTime;
};

typedef std::shared_ptr<SLAMModuleTriangulation> SLAMModuleTriangulationPtr;


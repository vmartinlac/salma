#pragma once

#include <memory>
#include "CameraCalibrationData.h"
#include "StereoRigCalibrationData.h"
#include "VideoReader.h"
#include "SLAMDataStructures.h"

enum SLAMErrorType
{
    SLAM_OK=0,
    SLAM_OTHER_ERROR
};

class SLAMSystem
{
public:

    static SLAMSystem* instance();

    ~SLAMSystem();

    bool initialize(int num_args, char** args);

    void run();

    void finalize();

protected:

    SLAMSystem();

    void printWelcomeMessage();

    bool parseCommandLineArguments(int num_args, char** args);

    void computeFeatures(FramePtr frame);

    void track(FramePtr frame);

    void map(FramePtr map);

    void globalBundleAdjustment();
    
protected:

    VideoReaderPtr mVideoReader;
    std::vector<MapPointPtr> mMapPoints;
    FramePtr mFirstFrame;
    FramePtr mLastFrame;
    CameraCalibrationData mCameraCalibration[2];
    StereoRigCalibrationData mStereoRigCalibration;
    static std::unique_ptr<SLAMSystem> mInstance;
};


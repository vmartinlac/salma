#pragma once

#include <memory>
#include "CameraCalibrationData.h"
#include "StereoRigCalibrationData.h"
#include "VideoReader.h"
#include "SLAMDataStructures.h"

class SLAMSystem
{
public:

    static SLAMSystem* instance();

    ~SLAMSystem();

    void run(int num_args, char** args);

protected:

    SLAMSystem();

    bool initialize(int num_args, char** args);
    void printWelcomeMessage();
    bool parseCommandLineArguments(int num_args, char** args);
    void setCurrentFrame(Image& image);
    void finalize();

protected:

    struct CameraRectification
    {
        cv::Mat R;
        cv::Mat P;
        cv::Mat map0;
        cv::Mat map1;
    };

    struct StereoRectification
    {
        CameraRectification camera[2];
        cv::Mat Q;
        cv::Mat R;
        cv::Mat T;
    };

protected:

    VideoSourcePtr mVideo;

    FramePtr mFirstFrame;
    FramePtr mCurrentFrame;

    CameraCalibrationDataPtr mCameraCalibration[2];
    StereoRigCalibrationDataPtr mStereoRigCalibration;

    StereoRectification mRectification;

    static std::unique_ptr<SLAMSystem> mInstance;
};


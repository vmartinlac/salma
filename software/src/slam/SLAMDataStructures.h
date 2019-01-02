#pragma once

/*
Left is index 0.
Right is index 1.
*/

#include <memory>
#include <list>
#include <opencv2/core.hpp>
#include <sophus/se3.hpp>
#include <Eigen/Eigen>
#include "StereoRigCalibrationData.h"
#include "RecordingHeader.h"

class SLAMMapPoint
{
public:

    SLAMMapPoint()
    {
        id = -1;
    }

    int id;
    Eigen::Vector3d position;
};

typedef std::shared_ptr<SLAMMapPoint> SLAMMapPointPtr;

enum SLAMProjectionType
{
    PROJECTION_MAPPED,
    PROJECTION_TRACKED
};

class SLAMProjection
{
public:
    SLAMMapPointPtr mappoint;
    cv::Point2f point;
    SLAMProjectionType type;
    int max_lifetime;
};

class SLAMView
{
public:

    cv::Mat image;

    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    std::vector<SLAMProjection> projections;
};

class SLAMFrame
{
public:

    SLAMFrame()
    {
        id = -1;
        timestamp = 0.0;
        aligned_wrt_previous_frame = false;
    }

    int id;
    double timestamp;
    SLAMView views[2];
    std::vector< std::pair<int,int> > stereo_matches;
    Sophus::SE3d frame_to_world;
    bool aligned_wrt_previous_frame;
};

typedef std::shared_ptr<SLAMFrame> SLAMFramePtr;

typedef std::vector<SLAMFramePtr> SLAMFramePtrVector;

class SLAMReconstruction
{
public:

    SLAMReconstruction()
    {
        id = -1;
    }

    int id;
    std::string name;
    std::string date;

    RecordingHeaderPtr recording;
    StereoRigCalibrationDataPtr rig;

    SLAMFramePtrVector frames;
};

typedef std::shared_ptr<SLAMReconstruction> SLAMReconstructionPtr;


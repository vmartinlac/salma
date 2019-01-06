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

class SLAMTrack
{
public:

    SLAMMapPointPtr mappoint;
};

class SLAMView
{
public:

    cv::Mat image;

    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    std::vector<SLAMTrack> tracks;
};

class SLAMFrame
{
public:

    SLAMFrame()
    {
        id = -1;
        rank_in_recording = -1;
        timestamp = 0.0;
        aligned_wrt_previous_frame = false;
    }

    int id;

    int rank_in_recording;
    double timestamp;

    SLAMView views[2];
    std::vector< std::pair<int,int> > stereo_matches;
    Sophus::SE3d frame_to_world;
    bool aligned_wrt_previous_frame;
};

typedef std::shared_ptr<SLAMFrame> SLAMFramePtr;

class SLAMSegment
{
public:

    std::vector<SLAMFramePtr> frames;
};

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

    std::vector<SLAMFramePtr> frames;

    std::vector<SLAMSegment> segments;

public:

    void buildSegments();
};

typedef std::shared_ptr<SLAMReconstruction> SLAMReconstructionPtr;


#pragma once

/*
Left is index 0.
Right is index 1.
*/

#include <memory>
#include <list>
#include <opencv2/core.hpp>
#ifdef SALMA_WITH_CUDA
#include <opencv2/core/cuda.hpp>
#endif
#include <sophus/se3.hpp>
#include <Eigen/Eigen>
#include "StereoRigCalibration.h"
#include "RecordingHeader.h"

class SLAMMapPoint
{
public:

    SLAMMapPoint();

    int id;

    int num_outlier_verdicts;
    int num_inlier_verdicts;

    int frame_id_of_creation;
    int frame_id_of_last_position_update;

    Eigen::Vector3d position;
    Eigen::Matrix<double, 3, 10> position_covariance; // covariance of (mappoint_position) wrt (mappoint_position, camera_position, camera_attitude).
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

#ifdef SALMA_WITH_CUDA
    cv::cuda::GpuMat d_image;
#endif

    std::vector<cv::KeyPoint> keypoints;
    std::vector<cv::Point2f> normalized;
    cv::Mat descriptors;
    std::vector<SLAMTrack> tracks;
};

class SLAMColoredPoint
{
public:

    cv::Point3f point;
    cv::Vec3f color;
};

class SLAMFrame
{
public:

    SLAMFrame();

    int id;

    int rank_in_recording;
    double timestamp;

    SLAMView views[2];
    std::vector< std::pair<int,int> > stereo_matches;

    bool aligned_wrt_previous_frame;

    Sophus::SE3d frame_to_world;
    Eigen::Matrix<double, 7, 7> pose_covariance;

    std::vector<SLAMColoredPoint> dense_cloud;
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
    StereoRigCalibrationPtr calibration;

    std::vector<SLAMFramePtr> frames;

    std::vector<SLAMSegment> segments;

public:

    void buildSegments();
};

typedef std::shared_ptr<SLAMReconstruction> SLAMReconstructionPtr;


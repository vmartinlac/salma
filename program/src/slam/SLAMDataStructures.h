#pragma once

/*
Left is index 0.
Right is index 1.
*/

#include <memory>
#include <opencv2/core.hpp>
#include <sophus/se3.hpp>
#include <Eigen/Eigen>

class Frame;
class MapPoint;

typedef std::shared_ptr<Frame> FramePtr;
typedef std::shared_ptr<MapPoint> MapPointPtr;

class MapPoint
{
public:

    MapPoint()
    {
        id = -1;
        track_count = 0;
    }

    int id;
    Eigen::Vector3d position;
    //Eigen::Matrix3d covariance;
    int track_count;
};

class Track
{
public:

    Track()
    {
        anterior_match = -1;
        posterior_match = -1;
        stereo_match = -1;
    }

    int anterior_match; // match in same view of previous keyframe.
    int posterior_match; // match in same view of next keyframe.
    int stereo_match; // match in opposite view of current keyframe.

    MapPointPtr mappoint;
};

class View
{
public:

    cv::Mat image;

    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    std::vector<Track> tracks;
};

class Frame
{
public:

    int id;
    double timestamp;
    View views[2];
    Sophus::SE3d frame_to_world;

    FramePtr previous_frame;
};


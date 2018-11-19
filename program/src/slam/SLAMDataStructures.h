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

    int id;
    Eigen::Vector3d position;
    Eigen::Matrix3d covariance;
    int track_count;
};

class Track
{
public:

    Track()
    {
        match_in_previous_frame = -1;
    }

    int match_in_previous_frame;
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

typedef std::vector< std::pair<int,int> > StereoMatching;

class Frame
{
public:

    int id;
    double timestamp;
    View views[2];
    StereoMatching stereo_matching;
    Sophus::SE3d world_to_frame;
    FramePtr previous_frame;
};


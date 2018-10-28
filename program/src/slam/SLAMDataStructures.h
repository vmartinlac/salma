#pragma once

/*
Left is index 0.
Right is index 1.
*/

#include <memory>
#include <opencv2/core.hpp>
#include <sophus/se3.hpp>
#include <Eigen/Eigen>

class MapPoint
{
public:
    int id;
    Eigen::Vector3d position;
    int track_count;
};

typedef std::shared_ptr<MapPoint> MapPointPtr;

class View
{
public:
    cv::Mat image;
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    std::vector<MapPointPtr> mappoints;
};

class Frame;

typedef std::shared_ptr<Frame> FramePtr;

class Frame
{
public:
    int id;
    double timestamp;
    View views[2];
    Sophus::SE3d world_to_frame;
    FramePtr previous_frame;
};


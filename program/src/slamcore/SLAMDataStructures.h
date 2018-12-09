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
    }

    int id;
    Eigen::Vector3d position;
};

enum ProjectionType
{
    PROJECTION_MAPPED,
    PROJECTION_TRACKED
};

class Projection
{
public:
    MapPointPtr mappoint;
    cv::Point2f point;
    ProjectionType type;
    int max_lifetime;
};

class View
{
public:

    cv::Mat image;

    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    std::vector<Projection> projections;
};

class Frame
{
public:

    Frame()
    {
        id = -1;
        timestamp = 0.0;
        aligned_wrt_previous_frame = false;
    }

    int id;
    double timestamp;
    View views[2];
    std::vector< std::pair<int,int> > stereo_matches;
    Sophus::SE3d frame_to_world;
    bool aligned_wrt_previous_frame;

    FramePtr previous_frame;
};


#pragma once

#include <vector>
#include <opencv2/core.hpp>

namespace target {

class KeyPointListAdapter;

class Detector
{
public:

    void run(const cv::Mat& image);

protected:

    struct Target
    {
        cv::Vec2f origin;
        cv::Vec2f directions[3];
    };

protected:

    void filter_keypoints();
    bool filter_keypoint(const cv::KeyPoint& kp);
    void detect_target();
    void compute_neighborhoods();

protected:

    struct Neighborhood
    {
        Neighborhood()
        {
            valid = false;
        }

        bool valid;
        size_t neighbors[8];
    };

protected:

    const cv::Mat* m_image;
    cv::Mat m_greyscale;
    cv::Mat m_thresh;
    std::vector<cv::KeyPoint> m_keypoints;
    std::vector<Neighborhood> m_neighbors;
};

}


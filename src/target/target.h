#pragma once

#include <vector>
#include <opencv2/core.hpp>
#include <nanoflann.hpp>

namespace target {

class KeyPointListAdapter;

class Detector
{
public:

    void run(const cv::Mat& image);

protected:

    typedef nanoflann::KDTreeSingleIndexAdaptor<
        nanoflann::L2_Simple_Adaptor<float, KeyPointListAdapter>,
        KeyPointListAdapter,
        2,
        size_t> KDTree;

    struct Target
    {
        cv::Vec2f origin;
        cv::Vec2f directions[3];
    };

protected:

    void filter_keypoints();
    bool filter_keypoint(const cv::KeyPoint& kp);
    void detect_target();

protected:

    const cv::Mat* m_image;
    cv::Mat m_greyscale;
    cv::Mat m_thresh;
    std::vector<cv::KeyPoint> m_keypoints;
    KDTree* m_kdtree;
};

}


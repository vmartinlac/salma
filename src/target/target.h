#pragma once

#include <memory>
#include <vector>
#include <opencv2/core.hpp>
#include <nanoflann.hpp>
#include "KeyPointListAdapter.h"

namespace target {

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
        bool filter_line(const cv::Point2f& A, const cv::Point2f& B);
        void build_kdtree();
        void detect_target();
        void find_k_nearest_neighbors(size_t index, size_t k, std::vector<size_t>& neighbors);

    protected:

        typedef nanoflann::KDTreeSingleIndexAdaptor<
            nanoflann::L2_Simple_Adaptor<float, KeyPointListAdapter>,
            KeyPointListAdapter,
            2,
            size_t> KDTree;

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
        std::unique_ptr<KeyPointListAdapter> m_kpl_adapter;
        std::unique_ptr<KDTree> m_kdtree;
        std::vector<Neighborhood> m_neighbors;
    };
}


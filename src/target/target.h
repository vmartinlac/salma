#pragma once

#include <algorithm>
#include <memory>
#include <vector>
#include <opencv2/core.hpp>
#include <nanoflann.hpp>

namespace target {

    class Detector
    {
    public:

        Detector();

        Detector(const Detector& o) = delete;

        void operator=(const Detector& o) = delete;

        /*
        \param[in] image input image.
        \param[out] samples found. This is a CV_32F array with one row per sample and 5 columns which are image-x, image-y, space-x, space-y, space-z.
        \return true if samples where found.
        */
        bool run(
            const cv::Mat& image,
            cv::Mat& samples);

    protected:

        enum KindOfLine
        {
            LINE_NONE=0,
            LINE_BW=1,
            LINE_WB=2
        };

        struct SamplePoint
        {
            SamplePoint() { clear(); }

            SamplePoint(const cv::KeyPoint& kp) : keypoint(kp) { clear(); }

            void clear()
            {
                num_neighbors = 0;

                std::fill(neighbors, neighbors+4, -1);
                std::fill(neighbor_types, neighbor_types+4, LINE_NONE);

                connected_component = -1;

                coords2d[0] = 0;
                coords2d[1] = 0;
            }

            cv::KeyPoint keypoint;

            int num_neighbors;
            int neighbors[4];
            KindOfLine neighbor_types[4];

            int connected_component;

            int coords2d[2];

            //cv::Point3f location_in_space;
        };

        class PointListAdapter
        {
        public:

            PointListAdapter(const std::vector<SamplePoint>* list) :
                m_points(list)
            {
                ;
            }

            inline size_t kdtree_get_point_count() const
            {
                return m_points->size();
            }

            inline float kdtree_get_pt(size_t index, int dim) const
            {
                cv::Vec2f pt = m_points->operator[](index).keypoint.pt;
                return pt[dim];
            }

            template<typename BBOX>
            inline bool kdtree_get_bbox(BBOX& bb) const
            {
                return false;
            }

        protected:

            const std::vector<SamplePoint>* m_points;
        };

        struct ConnectedComponent
        {
            ConnectedComponent(int seed_, int size_) : seed(seed_), size(size_) { }
            int seed;
            int size;
        };

    protected:

        void build_kdtree();
        void detect_corners();
        bool filter_keypoint(const cv::KeyPoint& kp);
        void detect_neighbors();
        void find_k_nearest_neighbors(int index, size_t k, std::vector<int>& neighbors);
        bool filter_line(const cv::Point2f& A, const cv::Point2f& B, KindOfLine& kind);
        void symmetrize();
        void compute_connected_components();
        int find_connected_component(int seed, int component);
        void orient_neighborhood(int idx, int pre, int post);

    protected:

        typedef nanoflann::KDTreeSingleIndexAdaptor<
            nanoflann::L2_Simple_Adaptor<float, PointListAdapter>,
            PointListAdapter,
            2,
            int> KDTree;

    protected:

        const cv::Mat* m_image; // the original input image.
        cv::Mat m_greyscale; // the input image converted to greyscale.
        cv::Mat m_thresh; // thresholded input image where we can discriminate colors of chessboard.
        cv::Mat m_debug_image; // output image for debugging purpose.
        std::vector<SamplePoint> m_points; // the sample points.
        PointListAdapter m_kpl_adapter; // the adapter which allows nanoflann to use our datastructure.
        std::unique_ptr<KDTree> m_kdtree; // the kdtree used for knn searches.
        std::vector<ConnectedComponent> m_connected_components;
    };
}


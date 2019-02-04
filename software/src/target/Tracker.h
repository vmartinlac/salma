#pragma once

#include <algorithm>
#include <array>
#include <memory>
#include <nanoflann.hpp>
#include <Eigen/Eigen>
#include "TrackerBase.h"

namespace target
{
    class Tracker : public TrackerBase
    {
    public:

        Tracker();

        bool track( const cv::Mat& image, bool absolute_pose=true ) override;

        void clear() override;

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
                neighbors[0] = -1;
                neighbors[1] = -1;
                neighbors[2] = -1;
                neighbors[3] = -1;

                neighbor_types[0] = LINE_NONE;
                neighbor_types[1] = LINE_NONE;
                neighbor_types[2] = LINE_NONE;
                neighbor_types[3] = LINE_NONE;

                connected_component = -1;

                coords2d[0] = 0;
                coords2d[1] = 0;
            }

            bool full_neighborhood()
            {
                return
                    neighbors[0] >= 0 &&
                    neighbors[1] >= 0 &&
                    neighbors[2] >= 0 &&
                    neighbors[3] >= 0 ;
            }

            cv::KeyPoint keypoint;

            int neighbors[4];
            KindOfLine neighbor_types[4]; // neighbors[i] < 0 <=> neighbor_types[i] == LINE_NONE

            int connected_component;

            int coords2d[2]; // this field is filled if and only if connected_component >= 0.
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

        void threshold();
        void build_kdtree();
        void detect_corners();
        bool filter_keypoint(const cv::KeyPoint& kp);
        void detect_neighbors();
        void find_k_nearest_neighbors(int index, size_t k, std::vector<int>& neighbors);
        bool filter_line(const cv::Point2f& A, const cv::Point2f& B, KindOfLine& kind);
        void symmetrize();
        void compute_connected_components();
        int find_connected_component(int seed, int component);
        void orient_myself(int idx); // requires point idx to have four neighbors.
        void orient_my_neighbor(int idx, int neigh_id);
        bool compute_absolute_orientation();
        bool find_cell(int point, std::array<int,4>& cell);
        bool find_cell_anticlockwise(int point, std::array<int,4>& cell);
        bool find_cell_clockwise(int point, std::array<int,4>& cell);
        bool filter_circle(const Eigen::Matrix3d& H);
        bool save_results();
        static int computeZSquaredToN(int i, int j);

    protected:

        typedef nanoflann::KDTreeSingleIndexAdaptor<
            nanoflann::L2_Simple_Adaptor<float, PointListAdapter>,
            PointListAdapter,
            2,
            int> KDTree;

    protected:

        double m_unit_length;

        const cv::Mat* m_image; // the original input image.
        cv::Mat m_greyscale; // the input image converted to greyscale.
        cv::Mat m_thresh; // thresholded input image where we can discriminate colors of chessboard.
        std::vector<SamplePoint> m_points; // the sample points.
        PointListAdapter m_kpl_adapter; // the adapter which allows nanoflann to use our datastructure.
        std::unique_ptr<KDTree> m_kdtree; // the kdtree used for knn searches.
        std::vector<ConnectedComponent> m_connected_components;
        int m_biggest_connected_component;
    };
}


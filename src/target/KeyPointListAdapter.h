#pragma once

#include <opencv2/core.hpp>
#include <vector>

namespace target {

    class KeyPointListAdapter
    {
    public:

        KeyPointListAdapter(const std::vector<cv::KeyPoint>* list) :
            m_key_points(list)
        {
            ;
        }

        inline size_t kdtree_get_point_count() const
        {
            return m_key_points->size();
        }

        inline float kdtree_get_pt(size_t index, int dim) const
        {
            cv::Vec2f pt = m_key_points->operator[](index).pt;
            return pt[dim];
        }

        template<typename BBOX>
        inline bool kdtree_get_bbox(BBOX& bb) const
        {
            return false;
        }

    protected:

        const std::vector<cv::KeyPoint>* m_key_points;
    };

}

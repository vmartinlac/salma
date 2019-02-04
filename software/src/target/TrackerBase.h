#pragma once

#include <vector>
#include <opencv2/core.hpp>

namespace target
{
    class TrackerBase
    {
    public:

        TrackerBase();

        TrackerBase(const TrackerBase& o) = delete;

        virtual ~TrackerBase();

        void operator=(const TrackerBase& o) = delete;

        void setUnitLength(double length);

        virtual bool track( const cv::Mat& image, bool absolute_pose=true ) = 0;

        bool found();
        const std::vector<cv::Point3f>& objectPoints();
        const std::vector<cv::Point2f>& imagePoints();
        const std::vector<int>& pointIds();

        virtual void clear();

    protected:

        double m_unit_length;

        bool m_found;
        std::vector<cv::Point3f> m_object_points;
        std::vector<cv::Point2f> m_image_points;
        std::vector<int> m_point_ids;
    };

}


#include "TrackerBase.h"

target::TrackerBase::TrackerBase()
{
    m_unit_length = 1.0;
    m_found = false;
}

target::TrackerBase::~TrackerBase()
{
}

void target::TrackerBase::setUnitLength(double length)
{
    m_unit_length = length;
}

bool target::TrackerBase::found()
{
    return m_found;
}

const std::vector<cv::Point3f>& target::TrackerBase::objectPoints()
{
    return m_object_points;
}

const std::vector<cv::Point2f>& target::TrackerBase::imagePoints()
{
    return m_image_points;
}

const std::vector<int>& target::TrackerBase::pointIds()
{
    return m_point_ids;
}

void target::TrackerBase::clear()
{
    m_found = false;
    m_point_ids.clear();
    m_image_points.clear();
    m_object_points.clear();
}


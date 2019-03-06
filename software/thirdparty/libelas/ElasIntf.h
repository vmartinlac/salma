
#pragma once

#include <opencv2/calib3d.hpp>

class ElasIntf : public cv::StereoMatcher
{
public:

    static cv::Ptr<ElasIntf> create();
};


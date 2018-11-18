#pragma once

#include <opencv2/core.hpp>
#include <vector>

struct SLAMPyramidLevel
{
    cv::Mat image;
    double scale;
};

typedef std::vector<SLAMPyramidLevel> SLAMPyramid;


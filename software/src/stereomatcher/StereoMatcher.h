#pragma once

#include <opencv2/calib3d.hpp>

class StereoMatcher : public cv::StereoMatcher
{
public:

    static StereoMatcher* create();

    void compute(cv::InputArray left, cv::InputArray right, cv::OutputArray disparity) override = 0;
};


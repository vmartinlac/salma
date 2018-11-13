#pragma once

#include <opencv2/core.hpp>
#include <vector>

namespace Debug
{
    void imshow(const cv::Mat& image);

    void plotkeypointsandmatches(
        const cv::Mat& left,
        const std::vector<cv::KeyPoint>& lkpts,
        const cv::Mat& right,
        const std::vector<cv::KeyPoint>& rkpts);
}


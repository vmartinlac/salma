#pragma once

#include <opencv2/core.hpp>
#include <vector>

namespace Debug
{
    void imshow(const cv::Mat& image);

    void stereoimshow(
        const cv::Mat& left,
        const cv::Mat& right,
        const std::vector<cv::KeyPoint>& lkpts = std::vector<cv::KeyPoint>(),
        const std::vector<cv::KeyPoint>& rkpts = std::vector<cv::KeyPoint>(),
        const std::vector< std::pair<int,int> >& matches = std::vector< std::pair<int,int> >() );
}


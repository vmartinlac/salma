
#pragma once

#include <opencv2/core.hpp>
#include <vector>

class PhotometricNormalizer
{
public:

    PhotometricNormalizer();

    void prepare(const cv::Mat& left_image, const cv::Mat& right_image);

    void operator()(const cv::Mat& left_image, const cv::Mat& right_image);

protected:

    std::vector<float> mRightToLeft[3];
};

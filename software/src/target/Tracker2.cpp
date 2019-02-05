#include <iostream>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <map>
#include "Tracker2.h"

target::Tracker2::Tracker2()
{
}

bool target::Tracker2::track( const cv::Mat& image, bool absolute_pose )
{
    cv::imwrite("debug_output/10_original.png", image);

    // Converting to greyscale.
    cv::Mat gray;
    cv::cvtColor(image, gray, CV_BGR2GRAY);
    cv::imwrite("debug_output/20_greyscale.png", gray);

    // threshold.
    cv::Mat thresh_a;
    cv::threshold(gray, thresh_a, 0, 255, cv::THRESH_OTSU|cv::THRESH_BINARY_INV);
    cv::imwrite("debug_output/30_threshold.png", thresh_a);

    cv::Mat thresh_b(thresh_a.size(), thresh_a.type());;
    std::transform(
        thresh_a.begin<uint8_t>(),
        thresh_a.end<uint8_t>(),
        thresh_b.begin<uint8_t>(),
        [] (uint8_t x) { return (x == 0) ? 1 : 0; } );

    cv::Mat dist_a;
    cv::Mat dist_b;
    cv::distanceTransform(thresh_a, dist_a, cv::DIST_L2, 5, CV_32F);
    cv::distanceTransform(thresh_b, dist_b, cv::DIST_L2, 5, CV_32F);

    return false;
}

void target::Tracker2::clear()
{
    TrackerBase::clear();
}


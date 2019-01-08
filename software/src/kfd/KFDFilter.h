#pragma once

#include <opencv2/core.hpp>
#include <Eigen/Eigen>
#include <vector>
#include <sophus/se3.hpp>

class KFDFilter
{
public:

    KFDFilter();
    ~KFDFilter();

    bool init();

    void filter( double timestamp, const std::vector<cv::Point2f>& projections, const std::vector<cv::Point3f>& landmarks );

    void getPose(Sophus::SE3d& pose);
    void getPoseCovariance(Eigen::Matrix<double, 7, 7>& sigma);
};


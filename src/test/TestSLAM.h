#pragma once

#include <opencv2/core.hpp>
#include <Eigen/Eigen>
#include <QtTest>

class TestSLAM : public QObject
{
    Q_OBJECT

private slots:

    void test_f();
    void test_h();

private:

    void finite_differences_f(const Eigen::VectorXd& X, double dt, Eigen::MatrixXd& J);
    void finite_differences_h(
        const Eigen::VectorXd& X,
        const cv::Mat& K,
        const cv::Mat& dist,
        double min_dist,
        const cv::Rect& viewport,
        std::vector<int>& lms,
        Eigen::VectorXd& h,
        Eigen::MatrixXd& J);
};


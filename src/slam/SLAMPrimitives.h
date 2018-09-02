#pragma once

#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>

class SLAMPrimitives
{
public:

    static void compute_f(
        const Eigen::VectorXd& X,
        double dt,
        Eigen::VectorXd& f,
        Eigen::SparseMatrix<double>& J);

    static void compute_h(
        const Eigen::VectorXd& X,
        const cv::Mat& camera_matrix,
        const cv::Mat& distortion_coefficients,
        double min_distance_to_camera,
        const cv::Rect& viewport,
        std::vector<int>& visible_landmarks,
        Eigen::VectorXd& h,
        Eigen::SparseMatrix<double>& J);

    static void computeJacobianOfWorld2CameraTransformation(
        const Eigen::VectorXd& X,
        int landmark,
        Eigen::Matrix<double, 3, 10>& J);
};


#pragma once

#include <Eigen/Eigen>
#include <sophus/se3.hpp>

class SLAMMath
{
public:

    /*
    Format of quaternion is (x, y, z, w).
    Pointers can be set to null to indicate that the corresponding value is not to be returned.
    */

    static Eigen::Vector4d convert(const Eigen::Quaterniond& from);

    static Eigen::Quaterniond convert(const Eigen::Vector4d& from);

    static void computeQuaternionVectorProduct(
        const Eigen::Vector4d& q,
        const Eigen::Vector3d& v,
        Eigen::Vector3d* result,
        Eigen::Matrix<double, 3, 7>* J);

    static void computeQuaternionQuaternionProduct(
        const Eigen::Vector4d& q,
        const Eigen::Vector4d& p,
        Eigen::Vector4d* result,
        Eigen::Matrix<double, 4, 8>* J);

    static void rigidTransform(
        const Sophus::SE3d& mu_transform,
        const Eigen::Matrix<double, 7, 7>& sigma_transform,
        const Eigen::Vector3d& mu_point,
        const Eigen::Matrix3d& sigma_point,
        Eigen::Vector3d& mu_transformed,
        Eigen::Matrix<double, 3, 10>& sigma_transformed);

    static void rigidTransform(
        const Sophus::SE3d& transform,
        const Eigen::Vector3d& point,
        Eigen::Vector3d& result,
        Eigen::Matrix<double, 10, 10>& J); // jacobian of (input_vector, translation, quaternion) -> (output_vector, translation, quaternion).

    static void computeJacobianOfQuaternionToRotationMatrix(
        const Eigen::Quaterniond& q,
        Eigen::Matrix<double, 9, 4>& J);
};


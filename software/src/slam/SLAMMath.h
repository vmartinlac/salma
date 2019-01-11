#pragma once

#include <Eigen/Eigen>

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
};


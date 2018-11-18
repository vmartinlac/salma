#include <Eigen/Eigen>
#include <opencv2/core/eigen.hpp>
#include "Misc.h"

Eigen::Matrix3d Misc::vectorialProductMatrix(const Eigen::Vector3d& v)
{
    /*
    b*z - c*y
    c*x - a*z
    a*y - b*x
    */

    Eigen::Matrix3d ret;
    ret <<
        0.0, -v.z(), v.y(),
        v.z(), 0.0, -v.x(),
        -v.y(), v.x(), 0.0;
        
    return ret;
}

Eigen::Matrix3d Misc::computeFundamentalMatrix( CameraCalibrationDataPtr left_camera, CameraCalibrationDataPtr right_camera, StereoRigCalibrationDataPtr stereo_rig )
{
    /*
        Return F such that :

        right_l = F * left_x

        left_l = transposed(F) * right_x

        transposed(right_x) * F * left_x = 0

        transposed(left_x) * transposed(F) * right_x = 0
    */

    const Sophus::SE3d transform = stereo_rig->right_camera_to_world.inverse() * stereo_rig->left_camera_to_world;

    const Eigen::Matrix<double, 3, 4> transform34 = transform.matrix3x4();

    const Eigen::Matrix3d R = transform34.leftCols<3>();
    const Eigen::Vector3d t = transform34.rightCols<1>();

    Eigen::Matrix3d left_K;
    cv::cv2eigen<double, 3, 3>( left_camera->calibration_matrix, left_K );

    Eigen::Matrix3d right_K;
    cv::cv2eigen<double, 3, 3>( right_camera->calibration_matrix, right_K );

    const Eigen::Matrix3d inverse_left_K = left_camera->inverseOfCalibrationMatrix();
    const Eigen::Matrix3d inverse_right_K = right_camera->inverseOfCalibrationMatrix();

    const Eigen::Matrix3d F = inverse_right_K.transpose() * R * left_K.transpose() * Misc::vectorialProductMatrix( left_K * R.transpose() * t );

    /*
    for(double l=1.0; l<100.0; l+=0.25)
    {
        const auto PL = stereo_rig->left_camera_to_world.matrix3x4();
        const auto PR = stereo_rig->right_camera_to_world.matrix3x4();

        const Eigen::Vector3d X = PL.rightCols<1>() + PL.col(2) * l;

        Eigen::Vector3d left_x = left_camera->calibrationMatrix() * ( stereo_rig->left_camera_to_world.inverse() * X );
        Eigen::Vector3d right_x = right_camera->calibrationMatrix() * ( stereo_rig->right_camera_to_world.inverse() * X );

        std::cout << right_x.transpose() * F * left_x << std::endl;
    }
    */

    return F;
}


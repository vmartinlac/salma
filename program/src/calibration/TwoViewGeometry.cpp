#include "TwoViewGeometry.h"

Eigen::Matrix3d TwoViewGeometry::vectorialProductMatrix(const Eigen::Vector3d& v)
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

Eigen::Matrix3d TwoViewGeometry::computeFundamentalMatrix(
    CameraCalibrationDataPtr left_camera,
    CameraCalibrationDataPtr right_camera,
    StereoRigCalibrationDataPtr stereo_rig )
{
    /*
    Return F such that transpose(left_image_point) * F * right_image_point is zero.
    */

    const Eigen::Matrix3d inverse_left_K = left_camera->inverseOfCalibrationMatrix();
    const Eigen::Matrix3d inverse_right_K = right_camera->inverseOfCalibrationMatrix();

    const Eigen::Matrix3d E = computeEssentialMatrix( left_camera, right_camera, stereo_rig );

    const Eigen::Matrix3d F = inverse_left_K.transpose() * E * inverse_right_K;

    /*
    {
        Eigen::Matrix3d KL;
        Eigen::Matrix3d KR;
        cv::cv2eigen( left_camera->calibration_matrix, KL );
        cv::cv2eigen( right_camera->calibration_matrix, KR );

        const Sophus::SE3d rig_to_left = stereo_rig->left_camera_to_rig.inverse();
        const Sophus::SE3d rig_to_right = stereo_rig->right_camera_to_rig.inverse();

        for(double l=100.0; l<800.0; l+=10.0)
        {
            const Eigen::Vector3d P = stereo_rig->left_camera_to_rig.translation() + l * stereo_rig->left_camera_to_rig.rotationMatrix().col(2);

            const Eigen::Vector3d XL = KL * (rig_to_left * P);
            const Eigen::Vector3d XR = KR * (rig_to_right * P);
        }
        exit(0);
    }
    */

    return F;
}

Eigen::Matrix3d TwoViewGeometry::computeEssentialMatrix(
    CameraCalibrationDataPtr left_camera,
    CameraCalibrationDataPtr right_camera,
    StereoRigCalibrationDataPtr stereo_rig )
{
    /*
    Return E such that transpose(left_image_point) * E * right_image_point is zero.
    */

    const Sophus::SE3d transform = stereo_rig->left_camera_to_rig.inverse() * stereo_rig->right_camera_to_rig;

    const Eigen::Matrix3d R = transform.rotationMatrix();
    const Eigen::Vector3d t = transform.translation();

    const Eigen::Matrix3d E = vectorialProductMatrix( t ) * R;

    return E;
}


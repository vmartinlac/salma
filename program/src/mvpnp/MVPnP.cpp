#include "MVPnP.h"
#include <lbfgs.h>

bool MVPnP::Solver::run( const std::vector<View>& views, Sophus::SE3d& rig_to_world, std::vector< std::vector<bool> >& inliers)
{
    return false;
}

/*

    void computeWorldToCamera(
        const Eigen::Vector3d& point_in_world,
        const Eigen::Vector3d& rig_to_camera_t,
        const Eigen::Matrix3d& rig_to_camera_R,
        const Eigen::Vector3d& world_to_rig_t,
        const Eigen::Quaternio,
        Eigen::Vector2d& point_in_camera,
        Eigen::Matrix<double, >& jac_wrt_);

    void projection(
        const cv::Mat& calibration_matrix,
        const cv::Mat& distortion_coefficients,
        const Eigen::Vector3d& point,
        Eigen::Vector2d& projection,
        Eigen::Matrix<double, >& jac_wrt_point);

*/

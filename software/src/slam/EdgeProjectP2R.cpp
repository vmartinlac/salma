#include <Eigen/Eigen>
#include <opencv2/core/eigen.hpp>
#include <opencv2/calib3d.hpp>
#include "SLAMMath.h"
#include "EdgeProjectP2R.h"

EdgeProjectP2R::EdgeProjectP2R()
{
    mView = -1;
}

void EdgeProjectP2R::setView(int view)
{
    mView = view;
}

void EdgeProjectP2R::setRigCalibration(StereoRigCalibrationPtr rig)
{
    mRigCalibration = std::move(rig);
}

bool EdgeProjectP2R::read(std::istream& is)
{
    is >> _measurement.x;
    is >> _measurement.y;

    for (int i = 0; i < 2; i++)
    {
        for (int j = i; j < 2; j++)
        {
            is >> information()(i, j);
            if (i != j)
            {
                information()(j, i) = information()(i, j);
            }
        }
    }

    return true;
}

bool EdgeProjectP2R::write(std::ostream& os) const
{
    os << measurement().x << " ";
    os << measurement().y << " ";

    for (int i = 0; i < 2; i++)
    {
        for (int j = i; j < 2; j++)
        {
            os << " " << information()(i, j);
        }
    }

    return os.good();
}

void EdgeProjectP2R::computeError()
{
    const g2o::VertexSBAPointXYZ* point = static_cast<const g2o::VertexSBAPointXYZ*>(_vertices[0]);

    const g2o::VertexSE3Expmap* pose = static_cast<const g2o::VertexSE3Expmap*>(_vertices[1]);

    const g2o::SE3Quat world_to_rig = pose->estimate();

    const Eigen::Vector3d in_world_frame = point->estimate();

    const Eigen::Vector3d in_camera_frame =
        mRigCalibration->cameras[mView].camera_to_rig.inverse() * (world_to_rig * in_world_frame);

    const std::vector<cv::Point3f> to_project{ cv::Point3f(in_camera_frame.x(), in_camera_frame.y(), in_camera_frame.z()) };

    std::vector<cv::Point2f> projected;

    cv::projectPoints(
        to_project,
        cv::Mat::zeros(3,1,CV_64F),
        cv::Mat::zeros(3,1,CV_64F),
        mRigCalibration->cameras[mView].calibration_matrix,
        mRigCalibration->cameras[mView].distortion_coefficients,
        projected);

    if(projected.size() != 1) throw std::logic_error("internal error");

    _error.x() = _measurement.x - projected.front().x;
    _error.y() = _measurement.y - projected.front().y;
}

void EdgeProjectP2R::linearizeOplus()
{
    const g2o::VertexSBAPointXYZ* point = static_cast<const g2o::VertexSBAPointXYZ*>(_vertices[0]);

    const g2o::VertexSE3Expmap* pose = static_cast<const g2o::VertexSE3Expmap*>(_vertices[1]);

    const g2o::SE3Quat world_to_rig = pose->estimate();

    const Eigen::Vector3d in_world_frame = point->estimate();

    const Eigen::Vector3d in_camera_frame = mRigCalibration->cameras[mView].camera_to_rig.inverse() * (world_to_rig * in_world_frame);

    const std::vector<cv::Point3f> to_project{ cv::Point3f(in_camera_frame.x(), in_camera_frame.y(), in_camera_frame.z()) };

    std::vector<cv::Point2f> projected;

    cv::Mat J;

    cv::projectPoints(
        to_project,
        cv::Mat::zeros(3,1,CV_64F),
        cv::Mat::zeros(3,1,CV_64F),
        mRigCalibration->cameras[mView].calibration_matrix,
        mRigCalibration->cameras[mView].distortion_coefficients,
        projected,
        J);

    if(projected.size() != 1) throw std::logic_error("internal error");

    Eigen::Matrix<double, 2, 3> J_proj;
    cv::cv2eigen(J(cv::Range::all(), cv::Range(3,6)), J_proj);

    Eigen::Matrix<double, 3, 9> W;
    W.setZero();
    W.block<1,3>(0,0) = in_world_frame.transpose();
    W.block<1,3>(1,3) = in_world_frame.transpose();
    W.block<1,3>(2,6) = in_world_frame.transpose();

    Eigen::Matrix<double, 9, 4> J_r;
    SLAMMath::computeJacobianOfQuaternionToRotationMatrix(world_to_rig.rotation(), J_r);

    const Eigen::Matrix3d R_rig_to_camera = mRigCalibration->cameras[mView].camera_to_rig.rotationMatrix().transpose();

    const Eigen::Matrix3d R_world_to_camera = R_rig_to_camera * world_to_rig.rotation().toRotationMatrix();

    // jacobian or error wrt mappoint position.
    _jacobianOplusXi = -J_proj * R_world_to_camera;

    // jacobian of error wrt world_to_rig_q.
    _jacobianOplusXj.leftCols<3>() = -J_proj * R_rig_to_camera * W * J_r.leftCols<3>();

    // jacobian of error wrt world_to_rig_t.
    _jacobianOplusXj.rightCols<3>() = -J_proj * R_rig_to_camera;
}


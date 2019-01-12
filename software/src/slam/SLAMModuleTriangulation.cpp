#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <Eigen/Eigen>
#include "SLAMModuleTriangulation.h"
#include "SLAMMath.h"

SLAMModuleTriangulation::SLAMModuleTriangulation(SLAMContextPtr con) : SLAMModule(con)
{
}

SLAMModuleTriangulation::~SLAMModuleTriangulation()
{
}

bool SLAMModuleTriangulation::init()
{
    SLAMContextPtr con = context();

    mLeftCamera = con->calibration->cameras[0].calibration;
    mRightCamera = con->calibration->cameras[1].calibration;
    mRig = con->calibration;

    mEssentialMatrix = mRig->computeEssentialMatrix(1, 0);
    mS << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0;
    mEssentialMatrixTilde = mS * mEssentialMatrix * mS.transpose();

    mMinDistanceToCamera = con->configuration->triangulation.min_distance_to_camera;
    mMinAngleBetweenRays = con->configuration->triangulation.min_angle_between_rays;
    mCheckPerpendicularLength = con->configuration->triangulation.check_perpendicular_length;
    mPerpendicularMaxLength = con->configuration->triangulation.perpendicular_max_length;
    mMaxReprojectionError = con->configuration->triangulation.max_reprojection_error;
    mInitialLifeTime = con->configuration->triangulation.track_lifetime;
    mUseLindstrom = con->configuration->triangulation.use_lindstrom;
    mSigmaProjLeft = con->configuration->triangulation.sigma_proj_left;
    mSigmaProjRight = con->configuration->triangulation.sigma_proj_right;

    return true;
}

void SLAMModuleTriangulation::operator()()
{
    std::cout << "   TRIANGULATION" << std::endl;

    SLAMReconstructionPtr reconstr = context()->reconstruction;

    if( reconstr->frames.empty() ) throw std::runtime_error("internal error");

    int triangulation_count = 0;

    SLAMFramePtr frame = reconstr->frames.back();

    for( std::pair<int,int>& p : frame->stereo_matches )
    {
        if( bool(frame->views[0].tracks[p.first].mappoint) == false || bool(frame->views[1].tracks[p.second].mappoint) == false )
        {

            SLAMMapPointPtr world_point = triangulate( frame, p.first, p.second );

            if(world_point)
            {
                if(bool(frame->views[0].tracks[p.first].mappoint) == false)
                {
                    frame->views[0].tracks[p.first].mappoint = world_point;
                }

                if(bool(frame->views[1].tracks[p.second].mappoint) == false)
                {
                    frame->views[1].tracks[p.second].mappoint = world_point;
                }

                triangulation_count++;
            }
        }
    }

    std::cout << "      Number of new mappoints: " << triangulation_count << std::endl;
}

SLAMMapPointPtr SLAMModuleTriangulation::triangulate(SLAMFramePtr frame, int left_keypoint, int right_keypoint)
{
    SLAMMapPointPtr ret;

    Eigen::Vector3d in_rig_frame;
    Eigen::Matrix3d cov_in_rig_frame;

    Eigen::Vector3d in_world_frame;
    Eigen::Matrix<double, 3, 10> cov_in_world_frame;

    const bool ok = triangulateInRigFrame(
        frame->views[0].keypoints[left_keypoint].pt,
        frame->views[1].keypoints[right_keypoint].pt,
        in_rig_frame,
        cov_in_rig_frame);

    if(ok)
    {
        SLAMMath::rigidTransform(
            frame->frame_to_world,
            frame->pose_covariance,
            in_rig_frame,
            cov_in_rig_frame,
            in_world_frame,
            cov_in_world_frame);
        /*
        std::cout << in_rig_frame << std::endl;
        std::cout << in_world_frame << std::endl;
        */
    }

    if(ok)
    {
        ret.reset(new SLAMMapPoint());

        ret->id = context()->num_mappoints;

        ret->position = in_world_frame;

        ret->position_covariance = cov_in_world_frame;

        ret->frame_id_of_creation = frame->id;

        ret->frame_id_of_last_position_update = frame->id;

        context()->num_mappoints++;
    }

    return ret;
}

void SLAMModuleTriangulation::correctWithLindstrom( Eigen::Vector3d& normalized_left, Eigen::Vector3d& normalized_right )
{
    // See "Triangulation Made Easy" from Peter Lindstrom, Lawrence Livermore National Laboratory.

    Eigen::Vector3d x_left_k = normalized_left;
    Eigen::Vector3d x_right_k = normalized_right;

    const Eigen::Vector2d n_left_1 = mS * mEssentialMatrix * normalized_right;
    const Eigen::Vector2d n_right_1 = mS * mEssentialMatrix.transpose() * normalized_left;

    for(int i=0; i<3; i++)
    {
        const Eigen::Vector2d n_left_k = mS * mEssentialMatrix * x_right_k;
        const Eigen::Vector2d n_right_k = mS * mEssentialMatrix.transpose() * x_left_k;

        const double a = n_left_k.transpose() * mEssentialMatrixTilde * n_right_k;

        const double b = 0.5 * ( n_left_1.dot(n_left_k) + n_right_1.dot(n_right_k) );
        //const double b = 0.5 * ( n_left_1.transpose()*n_left_k + n_right_1.transpose()*n_right_k );

        const double c = normalized_left.transpose() * mEssentialMatrix * normalized_right;

        const double d = std::sqrt( b*b - a*c );

        const double lambda = c / ( ( b > 0.0) ? (b+d) : (b-d) );

        x_left_k = normalized_left - lambda * (mS.transpose() * n_left_k);
        x_right_k = normalized_right - lambda * (mS.transpose() * n_right_k);
    }
}

bool SLAMModuleTriangulation::triangulateInRigFrame(
    const cv::Point2f& proj_left,
    const cv::Point2f& proj_right,
    Eigen::Vector3d& in_rig_frame,
    Eigen::Matrix3d& cov_in_rig_frame)
{
    // Declare some variables.

    bool ok = true;

    // Compute normalized points.

    const std::vector<cv::Point2f> distorted_left{ proj_left };
    const std::vector<cv::Point2f> distorted_right{ proj_right };

    std::vector<cv::Point2f> normalized_left_arr;
    std::vector<cv::Point2f> normalized_right_arr;

    cv::undistortPoints(distorted_left, normalized_left_arr, mLeftCamera->calibration_matrix, mLeftCamera->distortion_coefficients);
    cv::undistortPoints(distorted_right, normalized_right_arr, mRightCamera->calibration_matrix, mRightCamera->distortion_coefficients);

    Eigen::Vector3d normalized_left;
    normalized_left.x() = normalized_left_arr.front().x;
    normalized_left.y() = normalized_left_arr.front().y;
    normalized_left.z() = 1.0;

    Eigen::Vector3d normalized_right;
    normalized_right.x() = normalized_right_arr.front().x;
    normalized_right.y() = normalized_right_arr.front().y;
    normalized_right.z() = 1.0;

    // Computed corrected normalized points with Lindstrom method.

    if( mUseLindstrom )
    {
        //std::cout << normalized_left.transpose() << " " << normalized_right.transpose() << std::endl;
        correctWithLindstrom( normalized_left, normalized_right );
        //std::cout << normalized_left.transpose() << " " << normalized_right.transpose() << std::endl;
    }

    // Triangulate first in rig frame. It will be transformed to world frame at the very end of this function.
    // We use the middle of the common perpendicular of the two rays.

    const Eigen::Vector3d C0 = mRig->cameras[0].camera_to_rig.translation();
    const Eigen::Matrix3d R0 = mRig->cameras[0].camera_to_rig.rotationMatrix();

    const Eigen::Vector3d C1 = mRig->cameras[1].camera_to_rig.translation();
    const Eigen::Matrix3d R1 = mRig->cameras[1].camera_to_rig.rotationMatrix();

    Eigen::Vector3d D0 = R0 * normalized_left;
    Eigen::Vector3d D1 = R1 * normalized_right;
    D0.normalize();
    D1.normalize();

    Eigen::Matrix<double, 3, 2> M;
    M.leftCols<1>() = D0;
    M.rightCols<1>() = D1;

    Eigen::Matrix2d N;
    N << -1.0, 0.0, 0.0, 1.0;

    const Eigen::Matrix2d A = M.transpose() * M * N;
    const Eigen::Vector2d Y = M.transpose() * (C0 - C1);

    const double det = A(0,0) * A(1,1) - A(0,1) * A(1,0);

    const double max_det = std::min( std::pow(std::cos(mMinAngleBetweenRays), 2.0) - 1.0, -1.0e-8 );

    ok = ( det < max_det );

    if(ok)
    {
        ok = false;

        Eigen::Matrix2d invA;

        invA(0,0) = A(1,1)/det;
        invA(1,1) = A(0,0)/det;
        invA(0,1) = -A(0,1)/det;
        invA(1,0) = -A(1,0)/det;

        const Eigen::Vector2d coords = invA * Y;
        //std::cout << coords.transpose() << std::endl;

        if( coords(0) > mMinDistanceToCamera && coords(1) > mMinDistanceToCamera )
        {
            const Eigen::Vector3d X0 = C0 + coords(0) * D0;
            const Eigen::Vector3d X1 = C1 + coords(1) * D1;

            if( mCheckPerpendicularLength == false || (X1-X0).norm() < mPerpendicularMaxLength )
            {
                in_rig_frame = 0.5 * (X0 + X1);
                ok = true;
            }
        }
    }

    // Check reprojection.

    if(ok)
    {
        const Eigen::Vector3d in_left_camera = mRig->cameras[0].camera_to_rig.inverse() * in_rig_frame;
        const Eigen::Vector3d in_right_camera = mRig->cameras[1].camera_to_rig.inverse() * in_rig_frame;

        ok = (in_left_camera.z() > 0.0 && in_right_camera.z() > 0.0);

        if(ok)
        {
            cv::Point3f in_left_camera_cv;
            in_left_camera_cv.x = in_left_camera.x();
            in_left_camera_cv.y = in_left_camera.y();
            in_left_camera_cv.z = in_left_camera.z();

            cv::Point3f in_right_camera_cv;
            in_right_camera_cv.x = in_right_camera.x();
            in_right_camera_cv.y = in_right_camera.y();
            in_right_camera_cv.z = in_right_camera.z();

            std::vector<cv::Point3f> in_left_camera_arr{ in_left_camera_cv };
            std::vector<cv::Point3f> in_right_camera_arr{ in_right_camera_cv };

            std::vector<cv::Point2f> projected_left;
            std::vector<cv::Point2f> projected_right;

            cv::projectPoints(
                in_left_camera_arr,
                cv::Mat::zeros(3,1,CV_64F),
                cv::Mat::zeros(3,1,CV_64F),
                mLeftCamera->calibration_matrix,
                mLeftCamera->distortion_coefficients,
                projected_left);

            cv::projectPoints(
                in_right_camera_arr,
                cv::Mat::zeros(3,1,CV_64F),
                cv::Mat::zeros(3,1,CV_64F),
                mRightCamera->calibration_matrix,
                mRightCamera->distortion_coefficients,
                projected_right);

            const double error_left = cv::norm( projected_left.front() - distorted_left.front() );
            const double error_right = cv::norm( projected_right.front() - distorted_right.front() );

            //std::cout << error_left << " " << error_right << std::endl;

            ok = (error_left < mMaxReprojectionError && error_right < mMaxReprojectionError);
        }
    }

    // Compute covariance.

    if(ok)
    {
        Eigen::Matrix<double, 6, 4> J1;
        Eigen::Matrix<double, 3, 6> J2;

        {
            const double R1_11 = R0(0,0);
            const double R1_12 = R0(0,1);
            const double R1_13 = R0(0,2);
            const double R1_21 = R0(1,0);
            const double R1_22 = R0(1,1);
            const double R1_23 = R0(1,2);
            const double R1_31 = R0(2,0);
            const double R1_32 = R0(2,1);
            const double R1_33 = R0(2,2);

            const double R2_11 = R1(0,0);
            const double R2_12 = R1(0,1);
            const double R2_13 = R1(0,2);
            const double R2_21 = R1(1,0);
            const double R2_22 = R1(1,1);
            const double R2_23 = R1(1,2);
            const double R2_31 = R1(2,0);
            const double R2_32 = R1(2,1);
            const double R2_33 = R1(2,2);

            const double K1_fx = mLeftCamera->calibration_matrix.at<double>(0,0);
            const double K1_fy = mLeftCamera->calibration_matrix.at<double>(1,1);
            const double K1_cx = mLeftCamera->calibration_matrix.at<double>(0,2);
            const double K1_cy = mLeftCamera->calibration_matrix.at<double>(1,2);

            const double K2_fx = mRightCamera->calibration_matrix.at<double>(0,0);
            const double K2_fy = mRightCamera->calibration_matrix.at<double>(1,1);
            const double K2_cx = mRightCamera->calibration_matrix.at<double>(0,2);
            const double K2_cy = mRightCamera->calibration_matrix.at<double>(1,2);

            J1(0, 0) = R1_11/K1_fx;
            J1(0, 1) = R1_12/K1_fy;
            J1(0, 2) = 0;
            J1(0, 3) = 0;
            J1(1, 0) = R1_21/K1_fx;
            J1(1, 1) = R1_22/K1_fy;
            J1(1, 2) = 0;
            J1(1, 3) = 0;
            J1(2, 0) = R1_31/K1_fx;
            J1(2, 1) = R1_32/K1_fy;
            J1(2, 2) = 0;
            J1(2, 3) = 0;
            J1(3, 0) = 0;
            J1(3, 1) = 0;
            J1(3, 2) = R2_11/K2_fx;
            J1(3, 3) = R2_12/K2_fy;
            J1(4, 0) = 0;
            J1(4, 1) = 0;
            J1(4, 2) = R2_21/K2_fx;
            J1(4, 3) = R2_22/K2_fy;
            J1(5, 0) = 0;
            J1(5, 1) = 0;
            J1(5, 2) = R2_31/K2_fx;
            J1(5, 3) = R2_32/K2_fy;
        }

        {
            const double O1_x = C0.x();
            const double O1_y = C0.y();
            const double O1_z = C0.z();

            const double O2_x = C1.x();
            const double O2_y = C1.y();
            const double O2_z = C1.z();

            const double D1_x = D0.x();
            const double D1_y = D0.y();
            const double D1_z = D0.z();

            const double D2_x = D1.x();
            const double D2_y = D1.y();
            const double D2_z = D1.z();

            J2(0, 0) = (1.0/2.0)*D1_x*(2*D1_x*(D1_x*(-O1_x + O2_x) + D1_y*(-O1_y + O2_y) + D1_z*(-O1_z + O2_z))/((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2)) - D2_x*(D2_x*(-O1_x + O2_x) + D2_y*(-O1_y + O2_y) + D2_z*(-O1_z + O2_z))/((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2)) + (-O1_x + O2_x)*(pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))/((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2)) + (-2*D1_x*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) + 2*D2_x*(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z))*(pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(D1_x*(-O1_x + O2_x) + D1_y*(-O1_y + O2_y) + D1_z*(-O1_z + O2_z))/pow((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2), 2) + (-2*D1_x*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) + 2*D2_x*(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z))*(-D1_x*D2_x - D1_y*D2_y - D1_z*D2_z)*(D2_x*(-O1_x + O2_x) + D2_y*(-O1_y + O2_y) + D2_z*(-O1_z + O2_z))/pow((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2), 2)) - 1.0/2.0*D2_x*(-D2_x*(D1_x*(-O1_x + O2_x) + D1_y*(-O1_y + O2_y) + D1_z*(-O1_z + O2_z))/((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2)) + (-O1_x + O2_x)*(-D1_x*D2_x - D1_y*D2_y - D1_z*D2_z)/((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2)) + (-2*D1_x*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) + 2*D2_x*(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2))*(D2_x*(-O1_x + O2_x) + D2_y*(-O1_y + O2_y) + D2_z*(-O1_z + O2_z))/pow((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2), 2) + (-2*D1_x*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) + 2*D2_x*(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z))*(-D1_x*D2_x - D1_y*D2_y - D1_z*D2_z)*(D1_x*(-O1_x + O2_x) + D1_y*(-O1_y + O2_y) + D1_z*(-O1_z + O2_z))/pow((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2), 2)) + (1.0/2.0)*(pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(D1_x*(-O1_x + O2_x) + D1_y*(-O1_y + O2_y) + D1_z*(-O1_z + O2_z))/((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2)) + (1.0/2.0)*(-D1_x*D2_x - D1_y*D2_y - D1_z*D2_z)*(D2_x*(-O1_x + O2_x) + D2_y*(-O1_y + O2_y) + D2_z*(-O1_z + O2_z))/((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2));
            J2(0, 1) = (1.0/2.0)*D1_x*(2*D1_y*(D1_x*(-O1_x + O2_x) + D1_y*(-O1_y + O2_y) + D1_z*(-O1_z + O2_z))/((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2)) - D2_y*(D2_x*(-O1_x + O2_x) + D2_y*(-O1_y + O2_y) + D2_z*(-O1_z + O2_z))/((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2)) + (-O1_y + O2_y)*(pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))/((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2)) + (-2*D1_y*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) + 2*D2_y*(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z))*(pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(D1_x*(-O1_x + O2_x) + D1_y*(-O1_y + O2_y) + D1_z*(-O1_z + O2_z))/pow((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2), 2) + (-2*D1_y*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) + 2*D2_y*(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z))*(-D1_x*D2_x - D1_y*D2_y - D1_z*D2_z)*(D2_x*(-O1_x + O2_x) + D2_y*(-O1_y + O2_y) + D2_z*(-O1_z + O2_z))/pow((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2), 2)) - 1.0/2.0*D2_x*(-D2_y*(D1_x*(-O1_x + O2_x) + D1_y*(-O1_y + O2_y) + D1_z*(-O1_z + O2_z))/((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2)) + (-O1_y + O2_y)*(-D1_x*D2_x - D1_y*D2_y - D1_z*D2_z)/((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2)) + (-2*D1_y*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) + 2*D2_y*(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2))*(D2_x*(-O1_x + O2_x) + D2_y*(-O1_y + O2_y) + D2_z*(-O1_z + O2_z))/pow((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2), 2) + (-2*D1_y*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) + 2*D2_y*(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z))*(-D1_x*D2_x - D1_y*D2_y - D1_z*D2_z)*(D1_x*(-O1_x + O2_x) + D1_y*(-O1_y + O2_y) + D1_z*(-O1_z + O2_z))/pow((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2), 2));
            J2(0, 2) = (1.0/2.0)*D1_x*(2*D1_z*(D1_x*(-O1_x + O2_x) + D1_y*(-O1_y + O2_y) + D1_z*(-O1_z + O2_z))/((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2)) - D2_z*(D2_x*(-O1_x + O2_x) + D2_y*(-O1_y + O2_y) + D2_z*(-O1_z + O2_z))/((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2)) + (-O1_z + O2_z)*(pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))/((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2)) + (-2*D1_z*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) + 2*D2_z*(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z))*(pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(D1_x*(-O1_x + O2_x) + D1_y*(-O1_y + O2_y) + D1_z*(-O1_z + O2_z))/pow((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2), 2) + (-2*D1_z*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) + 2*D2_z*(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z))*(-D1_x*D2_x - D1_y*D2_y - D1_z*D2_z)*(D2_x*(-O1_x + O2_x) + D2_y*(-O1_y + O2_y) + D2_z*(-O1_z + O2_z))/pow((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2), 2)) - 1.0/2.0*D2_x*(-D2_z*(D1_x*(-O1_x + O2_x) + D1_y*(-O1_y + O2_y) + D1_z*(-O1_z + O2_z))/((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2)) + (-O1_z + O2_z)*(-D1_x*D2_x - D1_y*D2_y - D1_z*D2_z)/((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2)) + (-2*D1_z*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) + 2*D2_z*(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2))*(D2_x*(-O1_x + O2_x) + D2_y*(-O1_y + O2_y) + D2_z*(-O1_z + O2_z))/pow((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2), 2) + (-2*D1_z*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) + 2*D2_z*(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z))*(-D1_x*D2_x - D1_y*D2_y - D1_z*D2_z)*(D1_x*(-O1_x + O2_x) + D1_y*(-O1_y + O2_y) + D1_z*(-O1_z + O2_z))/pow((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2), 2));
            J2(0, 3) = (1.0/2.0)*D1_x*(-D1_x*(D2_x*(-O1_x + O2_x) + D2_y*(-O1_y + O2_y) + D2_z*(-O1_z + O2_z))/((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2)) + (-O1_x + O2_x)*(-D1_x*D2_x - D1_y*D2_y - D1_z*D2_z)/((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2)) + (2*D1_x*(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z) - 2*D2_x*(pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2)))*(pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(D1_x*(-O1_x + O2_x) + D1_y*(-O1_y + O2_y) + D1_z*(-O1_z + O2_z))/pow((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2), 2) + (2*D1_x*(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z) - 2*D2_x*(pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2)))*(-D1_x*D2_x - D1_y*D2_y - D1_z*D2_z)*(D2_x*(-O1_x + O2_x) + D2_y*(-O1_y + O2_y) + D2_z*(-O1_z + O2_z))/pow((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2), 2)) - 1.0/2.0*D2_x*(-D1_x*(D1_x*(-O1_x + O2_x) + D1_y*(-O1_y + O2_y) + D1_z*(-O1_z + O2_z))/((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2)) + 2*D2_x*(D2_x*(-O1_x + O2_x) + D2_y*(-O1_y + O2_y) + D2_z*(-O1_z + O2_z))/((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2)) + (-O1_x + O2_x)*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2))/((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2)) + (2*D1_x*(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z) - 2*D2_x*(pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2)))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2))*(D2_x*(-O1_x + O2_x) + D2_y*(-O1_y + O2_y) + D2_z*(-O1_z + O2_z))/pow((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2), 2) + (2*D1_x*(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z) - 2*D2_x*(pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2)))*(-D1_x*D2_x - D1_y*D2_y - D1_z*D2_z)*(D1_x*(-O1_x + O2_x) + D1_y*(-O1_y + O2_y) + D1_z*(-O1_z + O2_z))/pow((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2), 2)) - 1.0/2.0*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2))*(D2_x*(-O1_x + O2_x) + D2_y*(-O1_y + O2_y) + D2_z*(-O1_z + O2_z))/((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2)) - 1.0/2.0*(-D1_x*D2_x - D1_y*D2_y - D1_z*D2_z)*(D1_x*(-O1_x + O2_x) + D1_y*(-O1_y + O2_y) + D1_z*(-O1_z + O2_z))/((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2));
            J2(0, 4) = (1.0/2.0)*D1_x*(-D1_y*(D2_x*(-O1_x + O2_x) + D2_y*(-O1_y + O2_y) + D2_z*(-O1_z + O2_z))/((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2)) + (-O1_y + O2_y)*(-D1_x*D2_x - D1_y*D2_y - D1_z*D2_z)/((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2)) + (2*D1_y*(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z) - 2*D2_y*(pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2)))*(pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(D1_x*(-O1_x + O2_x) + D1_y*(-O1_y + O2_y) + D1_z*(-O1_z + O2_z))/pow((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2), 2) + (2*D1_y*(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z) - 2*D2_y*(pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2)))*(-D1_x*D2_x - D1_y*D2_y - D1_z*D2_z)*(D2_x*(-O1_x + O2_x) + D2_y*(-O1_y + O2_y) + D2_z*(-O1_z + O2_z))/pow((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2), 2)) - 1.0/2.0*D2_x*(-D1_y*(D1_x*(-O1_x + O2_x) + D1_y*(-O1_y + O2_y) + D1_z*(-O1_z + O2_z))/((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2)) + 2*D2_y*(D2_x*(-O1_x + O2_x) + D2_y*(-O1_y + O2_y) + D2_z*(-O1_z + O2_z))/((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2)) + (-O1_y + O2_y)*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2))/((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2)) + (2*D1_y*(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z) - 2*D2_y*(pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2)))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2))*(D2_x*(-O1_x + O2_x) + D2_y*(-O1_y + O2_y) + D2_z*(-O1_z + O2_z))/pow((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2), 2) + (2*D1_y*(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z) - 2*D2_y*(pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2)))*(-D1_x*D2_x - D1_y*D2_y - D1_z*D2_z)*(D1_x*(-O1_x + O2_x) + D1_y*(-O1_y + O2_y) + D1_z*(-O1_z + O2_z))/pow((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2), 2));
            J2(0, 5) = (1.0/2.0)*D1_x*(-D1_z*(D2_x*(-O1_x + O2_x) + D2_y*(-O1_y + O2_y) + D2_z*(-O1_z + O2_z))/((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2)) + (-O1_z + O2_z)*(-D1_x*D2_x - D1_y*D2_y - D1_z*D2_z)/((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2)) + (2*D1_z*(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z) - 2*D2_z*(pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2)))*(pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(D1_x*(-O1_x + O2_x) + D1_y*(-O1_y + O2_y) + D1_z*(-O1_z + O2_z))/pow((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2), 2) + (2*D1_z*(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z) - 2*D2_z*(pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2)))*(-D1_x*D2_x - D1_y*D2_y - D1_z*D2_z)*(D2_x*(-O1_x + O2_x) + D2_y*(-O1_y + O2_y) + D2_z*(-O1_z + O2_z))/pow((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2), 2)) - 1.0/2.0*D2_x*(-D1_z*(D1_x*(-O1_x + O2_x) + D1_y*(-O1_y + O2_y) + D1_z*(-O1_z + O2_z))/((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2)) + 2*D2_z*(D2_x*(-O1_x + O2_x) + D2_y*(-O1_y + O2_y) + D2_z*(-O1_z + O2_z))/((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2)) + (-O1_z + O2_z)*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2))/((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2)) + (2*D1_z*(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z) - 2*D2_z*(pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2)))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2))*(D2_x*(-O1_x + O2_x) + D2_y*(-O1_y + O2_y) + D2_z*(-O1_z + O2_z))/pow((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2), 2) + (2*D1_z*(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z) - 2*D2_z*(pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2)))*(-D1_x*D2_x - D1_y*D2_y - D1_z*D2_z)*(D1_x*(-O1_x + O2_x) + D1_y*(-O1_y + O2_y) + D1_z*(-O1_z + O2_z))/pow((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2), 2));
            J2(1, 0) = (1.0/2.0)*D1_y*(2*D1_x*(D1_x*(-O1_x + O2_x) + D1_y*(-O1_y + O2_y) + D1_z*(-O1_z + O2_z))/((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2)) - D2_x*(D2_x*(-O1_x + O2_x) + D2_y*(-O1_y + O2_y) + D2_z*(-O1_z + O2_z))/((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2)) + (-O1_x + O2_x)*(pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))/((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2)) + (-2*D1_x*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) + 2*D2_x*(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z))*(pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(D1_x*(-O1_x + O2_x) + D1_y*(-O1_y + O2_y) + D1_z*(-O1_z + O2_z))/pow((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2), 2) + (-2*D1_x*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) + 2*D2_x*(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z))*(-D1_x*D2_x - D1_y*D2_y - D1_z*D2_z)*(D2_x*(-O1_x + O2_x) + D2_y*(-O1_y + O2_y) + D2_z*(-O1_z + O2_z))/pow((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2), 2)) - 1.0/2.0*D2_y*(-D2_x*(D1_x*(-O1_x + O2_x) + D1_y*(-O1_y + O2_y) + D1_z*(-O1_z + O2_z))/((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2)) + (-O1_x + O2_x)*(-D1_x*D2_x - D1_y*D2_y - D1_z*D2_z)/((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2)) + (-2*D1_x*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) + 2*D2_x*(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2))*(D2_x*(-O1_x + O2_x) + D2_y*(-O1_y + O2_y) + D2_z*(-O1_z + O2_z))/pow((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2), 2) + (-2*D1_x*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) + 2*D2_x*(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z))*(-D1_x*D2_x - D1_y*D2_y - D1_z*D2_z)*(D1_x*(-O1_x + O2_x) + D1_y*(-O1_y + O2_y) + D1_z*(-O1_z + O2_z))/pow((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2), 2));
            J2(1, 1) = (1.0/2.0)*D1_y*(2*D1_y*(D1_x*(-O1_x + O2_x) + D1_y*(-O1_y + O2_y) + D1_z*(-O1_z + O2_z))/((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2)) - D2_y*(D2_x*(-O1_x + O2_x) + D2_y*(-O1_y + O2_y) + D2_z*(-O1_z + O2_z))/((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2)) + (-O1_y + O2_y)*(pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))/((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2)) + (-2*D1_y*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) + 2*D2_y*(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z))*(pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(D1_x*(-O1_x + O2_x) + D1_y*(-O1_y + O2_y) + D1_z*(-O1_z + O2_z))/pow((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2), 2) + (-2*D1_y*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) + 2*D2_y*(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z))*(-D1_x*D2_x - D1_y*D2_y - D1_z*D2_z)*(D2_x*(-O1_x + O2_x) + D2_y*(-O1_y + O2_y) + D2_z*(-O1_z + O2_z))/pow((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2), 2)) - 1.0/2.0*D2_y*(-D2_y*(D1_x*(-O1_x + O2_x) + D1_y*(-O1_y + O2_y) + D1_z*(-O1_z + O2_z))/((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2)) + (-O1_y + O2_y)*(-D1_x*D2_x - D1_y*D2_y - D1_z*D2_z)/((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2)) + (-2*D1_y*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) + 2*D2_y*(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2))*(D2_x*(-O1_x + O2_x) + D2_y*(-O1_y + O2_y) + D2_z*(-O1_z + O2_z))/pow((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2), 2) + (-2*D1_y*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) + 2*D2_y*(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z))*(-D1_x*D2_x - D1_y*D2_y - D1_z*D2_z)*(D1_x*(-O1_x + O2_x) + D1_y*(-O1_y + O2_y) + D1_z*(-O1_z + O2_z))/pow((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2), 2)) + (1.0/2.0)*(pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(D1_x*(-O1_x + O2_x) + D1_y*(-O1_y + O2_y) + D1_z*(-O1_z + O2_z))/((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2)) + (1.0/2.0)*(-D1_x*D2_x - D1_y*D2_y - D1_z*D2_z)*(D2_x*(-O1_x + O2_x) + D2_y*(-O1_y + O2_y) + D2_z*(-O1_z + O2_z))/((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2));
            J2(1, 2) = (1.0/2.0)*D1_y*(2*D1_z*(D1_x*(-O1_x + O2_x) + D1_y*(-O1_y + O2_y) + D1_z*(-O1_z + O2_z))/((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2)) - D2_z*(D2_x*(-O1_x + O2_x) + D2_y*(-O1_y + O2_y) + D2_z*(-O1_z + O2_z))/((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2)) + (-O1_z + O2_z)*(pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))/((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2)) + (-2*D1_z*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) + 2*D2_z*(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z))*(pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(D1_x*(-O1_x + O2_x) + D1_y*(-O1_y + O2_y) + D1_z*(-O1_z + O2_z))/pow((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2), 2) + (-2*D1_z*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) + 2*D2_z*(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z))*(-D1_x*D2_x - D1_y*D2_y - D1_z*D2_z)*(D2_x*(-O1_x + O2_x) + D2_y*(-O1_y + O2_y) + D2_z*(-O1_z + O2_z))/pow((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2), 2)) - 1.0/2.0*D2_y*(-D2_z*(D1_x*(-O1_x + O2_x) + D1_y*(-O1_y + O2_y) + D1_z*(-O1_z + O2_z))/((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2)) + (-O1_z + O2_z)*(-D1_x*D2_x - D1_y*D2_y - D1_z*D2_z)/((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2)) + (-2*D1_z*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) + 2*D2_z*(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2))*(D2_x*(-O1_x + O2_x) + D2_y*(-O1_y + O2_y) + D2_z*(-O1_z + O2_z))/pow((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2), 2) + (-2*D1_z*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) + 2*D2_z*(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z))*(-D1_x*D2_x - D1_y*D2_y - D1_z*D2_z)*(D1_x*(-O1_x + O2_x) + D1_y*(-O1_y + O2_y) + D1_z*(-O1_z + O2_z))/pow((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2), 2));
            J2(1, 3) = (1.0/2.0)*D1_y*(-D1_x*(D2_x*(-O1_x + O2_x) + D2_y*(-O1_y + O2_y) + D2_z*(-O1_z + O2_z))/((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2)) + (-O1_x + O2_x)*(-D1_x*D2_x - D1_y*D2_y - D1_z*D2_z)/((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2)) + (2*D1_x*(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z) - 2*D2_x*(pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2)))*(pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(D1_x*(-O1_x + O2_x) + D1_y*(-O1_y + O2_y) + D1_z*(-O1_z + O2_z))/pow((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2), 2) + (2*D1_x*(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z) - 2*D2_x*(pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2)))*(-D1_x*D2_x - D1_y*D2_y - D1_z*D2_z)*(D2_x*(-O1_x + O2_x) + D2_y*(-O1_y + O2_y) + D2_z*(-O1_z + O2_z))/pow((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2), 2)) - 1.0/2.0*D2_y*(-D1_x*(D1_x*(-O1_x + O2_x) + D1_y*(-O1_y + O2_y) + D1_z*(-O1_z + O2_z))/((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2)) + 2*D2_x*(D2_x*(-O1_x + O2_x) + D2_y*(-O1_y + O2_y) + D2_z*(-O1_z + O2_z))/((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2)) + (-O1_x + O2_x)*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2))/((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2)) + (2*D1_x*(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z) - 2*D2_x*(pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2)))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2))*(D2_x*(-O1_x + O2_x) + D2_y*(-O1_y + O2_y) + D2_z*(-O1_z + O2_z))/pow((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2), 2) + (2*D1_x*(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z) - 2*D2_x*(pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2)))*(-D1_x*D2_x - D1_y*D2_y - D1_z*D2_z)*(D1_x*(-O1_x + O2_x) + D1_y*(-O1_y + O2_y) + D1_z*(-O1_z + O2_z))/pow((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2), 2));
            J2(1, 4) = (1.0/2.0)*D1_y*(-D1_y*(D2_x*(-O1_x + O2_x) + D2_y*(-O1_y + O2_y) + D2_z*(-O1_z + O2_z))/((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2)) + (-O1_y + O2_y)*(-D1_x*D2_x - D1_y*D2_y - D1_z*D2_z)/((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2)) + (2*D1_y*(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z) - 2*D2_y*(pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2)))*(pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(D1_x*(-O1_x + O2_x) + D1_y*(-O1_y + O2_y) + D1_z*(-O1_z + O2_z))/pow((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2), 2) + (2*D1_y*(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z) - 2*D2_y*(pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2)))*(-D1_x*D2_x - D1_y*D2_y - D1_z*D2_z)*(D2_x*(-O1_x + O2_x) + D2_y*(-O1_y + O2_y) + D2_z*(-O1_z + O2_z))/pow((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2), 2)) - 1.0/2.0*D2_y*(-D1_y*(D1_x*(-O1_x + O2_x) + D1_y*(-O1_y + O2_y) + D1_z*(-O1_z + O2_z))/((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2)) + 2*D2_y*(D2_x*(-O1_x + O2_x) + D2_y*(-O1_y + O2_y) + D2_z*(-O1_z + O2_z))/((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2)) + (-O1_y + O2_y)*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2))/((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2)) + (2*D1_y*(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z) - 2*D2_y*(pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2)))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2))*(D2_x*(-O1_x + O2_x) + D2_y*(-O1_y + O2_y) + D2_z*(-O1_z + O2_z))/pow((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2), 2) + (2*D1_y*(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z) - 2*D2_y*(pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2)))*(-D1_x*D2_x - D1_y*D2_y - D1_z*D2_z)*(D1_x*(-O1_x + O2_x) + D1_y*(-O1_y + O2_y) + D1_z*(-O1_z + O2_z))/pow((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2), 2)) - 1.0/2.0*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2))*(D2_x*(-O1_x + O2_x) + D2_y*(-O1_y + O2_y) + D2_z*(-O1_z + O2_z))/((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2)) - 1.0/2.0*(-D1_x*D2_x - D1_y*D2_y - D1_z*D2_z)*(D1_x*(-O1_x + O2_x) + D1_y*(-O1_y + O2_y) + D1_z*(-O1_z + O2_z))/((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2));
            J2(1, 5) = (1.0/2.0)*D1_y*(-D1_z*(D2_x*(-O1_x + O2_x) + D2_y*(-O1_y + O2_y) + D2_z*(-O1_z + O2_z))/((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2)) + (-O1_z + O2_z)*(-D1_x*D2_x - D1_y*D2_y - D1_z*D2_z)/((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2)) + (2*D1_z*(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z) - 2*D2_z*(pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2)))*(pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(D1_x*(-O1_x + O2_x) + D1_y*(-O1_y + O2_y) + D1_z*(-O1_z + O2_z))/pow((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2), 2) + (2*D1_z*(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z) - 2*D2_z*(pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2)))*(-D1_x*D2_x - D1_y*D2_y - D1_z*D2_z)*(D2_x*(-O1_x + O2_x) + D2_y*(-O1_y + O2_y) + D2_z*(-O1_z + O2_z))/pow((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2), 2)) - 1.0/2.0*D2_y*(-D1_z*(D1_x*(-O1_x + O2_x) + D1_y*(-O1_y + O2_y) + D1_z*(-O1_z + O2_z))/((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2)) + 2*D2_z*(D2_x*(-O1_x + O2_x) + D2_y*(-O1_y + O2_y) + D2_z*(-O1_z + O2_z))/((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2)) + (-O1_z + O2_z)*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2))/((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2)) + (2*D1_z*(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z) - 2*D2_z*(pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2)))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2))*(D2_x*(-O1_x + O2_x) + D2_y*(-O1_y + O2_y) + D2_z*(-O1_z + O2_z))/pow((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2), 2) + (2*D1_z*(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z) - 2*D2_z*(pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2)))*(-D1_x*D2_x - D1_y*D2_y - D1_z*D2_z)*(D1_x*(-O1_x + O2_x) + D1_y*(-O1_y + O2_y) + D1_z*(-O1_z + O2_z))/pow((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2), 2));
            J2(2, 0) = (1.0/2.0)*D1_z*(2*D1_x*(D1_x*(-O1_x + O2_x) + D1_y*(-O1_y + O2_y) + D1_z*(-O1_z + O2_z))/((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2)) - D2_x*(D2_x*(-O1_x + O2_x) + D2_y*(-O1_y + O2_y) + D2_z*(-O1_z + O2_z))/((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2)) + (-O1_x + O2_x)*(pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))/((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2)) + (-2*D1_x*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) + 2*D2_x*(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z))*(pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(D1_x*(-O1_x + O2_x) + D1_y*(-O1_y + O2_y) + D1_z*(-O1_z + O2_z))/pow((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2), 2) + (-2*D1_x*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) + 2*D2_x*(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z))*(-D1_x*D2_x - D1_y*D2_y - D1_z*D2_z)*(D2_x*(-O1_x + O2_x) + D2_y*(-O1_y + O2_y) + D2_z*(-O1_z + O2_z))/pow((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2), 2)) - 1.0/2.0*D2_z*(-D2_x*(D1_x*(-O1_x + O2_x) + D1_y*(-O1_y + O2_y) + D1_z*(-O1_z + O2_z))/((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2)) + (-O1_x + O2_x)*(-D1_x*D2_x - D1_y*D2_y - D1_z*D2_z)/((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2)) + (-2*D1_x*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) + 2*D2_x*(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2))*(D2_x*(-O1_x + O2_x) + D2_y*(-O1_y + O2_y) + D2_z*(-O1_z + O2_z))/pow((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2), 2) + (-2*D1_x*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) + 2*D2_x*(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z))*(-D1_x*D2_x - D1_y*D2_y - D1_z*D2_z)*(D1_x*(-O1_x + O2_x) + D1_y*(-O1_y + O2_y) + D1_z*(-O1_z + O2_z))/pow((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2), 2));
            J2(2, 1) = (1.0/2.0)*D1_z*(2*D1_y*(D1_x*(-O1_x + O2_x) + D1_y*(-O1_y + O2_y) + D1_z*(-O1_z + O2_z))/((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2)) - D2_y*(D2_x*(-O1_x + O2_x) + D2_y*(-O1_y + O2_y) + D2_z*(-O1_z + O2_z))/((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2)) + (-O1_y + O2_y)*(pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))/((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2)) + (-2*D1_y*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) + 2*D2_y*(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z))*(pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(D1_x*(-O1_x + O2_x) + D1_y*(-O1_y + O2_y) + D1_z*(-O1_z + O2_z))/pow((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2), 2) + (-2*D1_y*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) + 2*D2_y*(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z))*(-D1_x*D2_x - D1_y*D2_y - D1_z*D2_z)*(D2_x*(-O1_x + O2_x) + D2_y*(-O1_y + O2_y) + D2_z*(-O1_z + O2_z))/pow((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2), 2)) - 1.0/2.0*D2_z*(-D2_y*(D1_x*(-O1_x + O2_x) + D1_y*(-O1_y + O2_y) + D1_z*(-O1_z + O2_z))/((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2)) + (-O1_y + O2_y)*(-D1_x*D2_x - D1_y*D2_y - D1_z*D2_z)/((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2)) + (-2*D1_y*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) + 2*D2_y*(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2))*(D2_x*(-O1_x + O2_x) + D2_y*(-O1_y + O2_y) + D2_z*(-O1_z + O2_z))/pow((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2), 2) + (-2*D1_y*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) + 2*D2_y*(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z))*(-D1_x*D2_x - D1_y*D2_y - D1_z*D2_z)*(D1_x*(-O1_x + O2_x) + D1_y*(-O1_y + O2_y) + D1_z*(-O1_z + O2_z))/pow((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2), 2));
            J2(2, 2) = (1.0/2.0)*D1_z*(2*D1_z*(D1_x*(-O1_x + O2_x) + D1_y*(-O1_y + O2_y) + D1_z*(-O1_z + O2_z))/((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2)) - D2_z*(D2_x*(-O1_x + O2_x) + D2_y*(-O1_y + O2_y) + D2_z*(-O1_z + O2_z))/((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2)) + (-O1_z + O2_z)*(pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))/((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2)) + (-2*D1_z*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) + 2*D2_z*(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z))*(pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(D1_x*(-O1_x + O2_x) + D1_y*(-O1_y + O2_y) + D1_z*(-O1_z + O2_z))/pow((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2), 2) + (-2*D1_z*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) + 2*D2_z*(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z))*(-D1_x*D2_x - D1_y*D2_y - D1_z*D2_z)*(D2_x*(-O1_x + O2_x) + D2_y*(-O1_y + O2_y) + D2_z*(-O1_z + O2_z))/pow((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2), 2)) - 1.0/2.0*D2_z*(-D2_z*(D1_x*(-O1_x + O2_x) + D1_y*(-O1_y + O2_y) + D1_z*(-O1_z + O2_z))/((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2)) + (-O1_z + O2_z)*(-D1_x*D2_x - D1_y*D2_y - D1_z*D2_z)/((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2)) + (-2*D1_z*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) + 2*D2_z*(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2))*(D2_x*(-O1_x + O2_x) + D2_y*(-O1_y + O2_y) + D2_z*(-O1_z + O2_z))/pow((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2), 2) + (-2*D1_z*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) + 2*D2_z*(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z))*(-D1_x*D2_x - D1_y*D2_y - D1_z*D2_z)*(D1_x*(-O1_x + O2_x) + D1_y*(-O1_y + O2_y) + D1_z*(-O1_z + O2_z))/pow((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2), 2)) + (1.0/2.0)*(pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(D1_x*(-O1_x + O2_x) + D1_y*(-O1_y + O2_y) + D1_z*(-O1_z + O2_z))/((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2)) + (1.0/2.0)*(-D1_x*D2_x - D1_y*D2_y - D1_z*D2_z)*(D2_x*(-O1_x + O2_x) + D2_y*(-O1_y + O2_y) + D2_z*(-O1_z + O2_z))/((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2));
            J2(2, 3) = (1.0/2.0)*D1_z*(-D1_x*(D2_x*(-O1_x + O2_x) + D2_y*(-O1_y + O2_y) + D2_z*(-O1_z + O2_z))/((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2)) + (-O1_x + O2_x)*(-D1_x*D2_x - D1_y*D2_y - D1_z*D2_z)/((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2)) + (2*D1_x*(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z) - 2*D2_x*(pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2)))*(pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(D1_x*(-O1_x + O2_x) + D1_y*(-O1_y + O2_y) + D1_z*(-O1_z + O2_z))/pow((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2), 2) + (2*D1_x*(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z) - 2*D2_x*(pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2)))*(-D1_x*D2_x - D1_y*D2_y - D1_z*D2_z)*(D2_x*(-O1_x + O2_x) + D2_y*(-O1_y + O2_y) + D2_z*(-O1_z + O2_z))/pow((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2), 2)) - 1.0/2.0*D2_z*(-D1_x*(D1_x*(-O1_x + O2_x) + D1_y*(-O1_y + O2_y) + D1_z*(-O1_z + O2_z))/((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2)) + 2*D2_x*(D2_x*(-O1_x + O2_x) + D2_y*(-O1_y + O2_y) + D2_z*(-O1_z + O2_z))/((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2)) + (-O1_x + O2_x)*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2))/((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2)) + (2*D1_x*(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z) - 2*D2_x*(pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2)))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2))*(D2_x*(-O1_x + O2_x) + D2_y*(-O1_y + O2_y) + D2_z*(-O1_z + O2_z))/pow((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2), 2) + (2*D1_x*(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z) - 2*D2_x*(pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2)))*(-D1_x*D2_x - D1_y*D2_y - D1_z*D2_z)*(D1_x*(-O1_x + O2_x) + D1_y*(-O1_y + O2_y) + D1_z*(-O1_z + O2_z))/pow((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2), 2));
            J2(2, 4) = (1.0/2.0)*D1_z*(-D1_y*(D2_x*(-O1_x + O2_x) + D2_y*(-O1_y + O2_y) + D2_z*(-O1_z + O2_z))/((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2)) + (-O1_y + O2_y)*(-D1_x*D2_x - D1_y*D2_y - D1_z*D2_z)/((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2)) + (2*D1_y*(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z) - 2*D2_y*(pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2)))*(pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(D1_x*(-O1_x + O2_x) + D1_y*(-O1_y + O2_y) + D1_z*(-O1_z + O2_z))/pow((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2), 2) + (2*D1_y*(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z) - 2*D2_y*(pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2)))*(-D1_x*D2_x - D1_y*D2_y - D1_z*D2_z)*(D2_x*(-O1_x + O2_x) + D2_y*(-O1_y + O2_y) + D2_z*(-O1_z + O2_z))/pow((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2), 2)) - 1.0/2.0*D2_z*(-D1_y*(D1_x*(-O1_x + O2_x) + D1_y*(-O1_y + O2_y) + D1_z*(-O1_z + O2_z))/((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2)) + 2*D2_y*(D2_x*(-O1_x + O2_x) + D2_y*(-O1_y + O2_y) + D2_z*(-O1_z + O2_z))/((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2)) + (-O1_y + O2_y)*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2))/((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2)) + (2*D1_y*(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z) - 2*D2_y*(pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2)))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2))*(D2_x*(-O1_x + O2_x) + D2_y*(-O1_y + O2_y) + D2_z*(-O1_z + O2_z))/pow((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2), 2) + (2*D1_y*(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z) - 2*D2_y*(pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2)))*(-D1_x*D2_x - D1_y*D2_y - D1_z*D2_z)*(D1_x*(-O1_x + O2_x) + D1_y*(-O1_y + O2_y) + D1_z*(-O1_z + O2_z))/pow((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2), 2));
            J2(2, 5) = (1.0/2.0)*D1_z*(-D1_z*(D2_x*(-O1_x + O2_x) + D2_y*(-O1_y + O2_y) + D2_z*(-O1_z + O2_z))/((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2)) + (-O1_z + O2_z)*(-D1_x*D2_x - D1_y*D2_y - D1_z*D2_z)/((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2)) + (2*D1_z*(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z) - 2*D2_z*(pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2)))*(pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(D1_x*(-O1_x + O2_x) + D1_y*(-O1_y + O2_y) + D1_z*(-O1_z + O2_z))/pow((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2), 2) + (2*D1_z*(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z) - 2*D2_z*(pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2)))*(-D1_x*D2_x - D1_y*D2_y - D1_z*D2_z)*(D2_x*(-O1_x + O2_x) + D2_y*(-O1_y + O2_y) + D2_z*(-O1_z + O2_z))/pow((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2), 2)) - 1.0/2.0*D2_z*(-D1_z*(D1_x*(-O1_x + O2_x) + D1_y*(-O1_y + O2_y) + D1_z*(-O1_z + O2_z))/((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2)) + 2*D2_z*(D2_x*(-O1_x + O2_x) + D2_y*(-O1_y + O2_y) + D2_z*(-O1_z + O2_z))/((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2)) + (-O1_z + O2_z)*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2))/((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2)) + (2*D1_z*(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z) - 2*D2_z*(pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2)))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2))*(D2_x*(-O1_x + O2_x) + D2_y*(-O1_y + O2_y) + D2_z*(-O1_z + O2_z))/pow((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2), 2) + (2*D1_z*(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z) - 2*D2_z*(pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2)))*(-D1_x*D2_x - D1_y*D2_y - D1_z*D2_z)*(D1_x*(-O1_x + O2_x) + D1_y*(-O1_y + O2_y) + D1_z*(-O1_z + O2_z))/pow((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2), 2)) - 1.0/2.0*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2))*(D2_x*(-O1_x + O2_x) + D2_y*(-O1_y + O2_y) + D2_z*(-O1_z + O2_z))/((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2)) - 1.0/2.0*(-D1_x*D2_x - D1_y*D2_y - D1_z*D2_z)*(D1_x*(-O1_x + O2_x) + D1_y*(-O1_y + O2_y) + D1_z*(-O1_z + O2_z))/((pow(D1_x, 2) + pow(D1_y, 2) + pow(D1_z, 2))*(pow(D2_x, 2) + pow(D2_y, 2) + pow(D2_z, 2)) - pow(D1_x*D2_x + D1_y*D2_y + D1_z*D2_z, 2));
        }

        Eigen::Matrix<double, 3, 4> J = J2 * J1;

        Eigen::Matrix<double, 4, 4> sigma_input;
        sigma_input.setZero();
        sigma_input(0,0) = mSigmaProjLeft*mSigmaProjLeft;
        sigma_input(1,1) = mSigmaProjLeft*mSigmaProjLeft;
        sigma_input(2,2) = mSigmaProjRight*mSigmaProjRight;
        sigma_input(3,3) = mSigmaProjRight*mSigmaProjRight;

        cov_in_rig_frame = J * sigma_input * J.transpose();
    }

    // If triangulation failed, clear output.

    if(ok == false)
    {
        in_rig_frame.setZero();
        cov_in_rig_frame.setZero();
    }

    return ok;
}

/*
void SLAMModuleTriangulation::computePositionCovariance(SLAMFramePtr frame, const Eigen::Vector3d& normalized_left, const Eigen::Vector3d& normalized_right, Eigen::Matrix<double, 3, 10>& cov)
{
    const Sophus::SE3d& leftcam_to_world = frame->frame_to_world * mRig->cameras[0].camera_to_rig;
    const Sophus::SE3d& rightcam_to_world = frame->frame_to_world * mRig->cameras[1].camera_to_rig;

    const Eigen::Vector3d dir_left = leftcam_to_world.so3() * normalized_left;
    const Eigen::Vector3d dir_right = rightcam_to_world.so3() * normalized_right;

    // jacobian of (rig2world_t, rig2world_q) -> (leftcam2world_t, leftcam2world_q, rightcam2world_t, rightcam2world_q)

    Eigen::Matrix<double, 14, 7> JB;

    {
        const Eigen::Vector4d leftcam_to_rig_q = SLAMMath::convert( mRig->cameras[0].camera_to_rig.unit_quaternion() );
        const Eigen::Vector4d rightcam_to_rig_q = SLAMMath::convert( mRig->cameras[1].camera_to_rig.unit_quaternion() );
        const Eigen::Vector4d rig_to_world_q = SLAMMath::convert( frame->frame_to_world.unit_quaternion() );

        Eigen::Matrix<double, 4, 8> JLQQ;
        Eigen::Matrix<double, 4, 8> JRQQ;
        Eigen::Matrix<double, 3, 7> JLQV;
        Eigen::Matrix<double, 3, 7> JRQV;

        SLAMMath::computeQuaternionQuaternionProduct(
            SLAMMath::convert(frame->frame_to_world.unit_quaternion()), 
            SLAMMath::convert(mRig->cameras[0].camera_to_rig.unit_quaternion()),
            nullptr, &JLQQ);

        SLAMMath::computeQuaternionQuaternionProduct(
            SLAMMath::convert(frame->frame_to_world.unit_quaternion()), 
            SLAMMath::convert(mRig->cameras[1].camera_to_rig.unit_quaternion()),
            nullptr, &JRQQ);

        SLAMMath::computeQuaternionVectorProduct(
            SLAMMath::convert(frame->frame_to_world.unit_quaternion()),
            mRig->cameras[0].camera_to_rig.translation(),
            nullptr, &JLQV);

        SLAMMath::computeQuaternionVectorProduct(
            SLAMMath::convert(frame->frame_to_world.unit_quaternion()),
            mRig->cameras[1].camera_to_rig.translation(),
            nullptr, &JRQV);

        JB.setZero();

        JB.block<3,3>(0,0).setIdentity();
        JB.block<3,4>(0,3) = JLQV.leftCols<4>();

        JB.block<4,3>(3,0).setZero();
        JB.block<4,4>(3,3) = JLQQ.leftCols<4>();

        JB.block<3,3>(7,0).setIdentity();
        JB.block<3,4>(7,3) = JRQV.leftCols<4>();

        JB.block<4,3>(10,0).setZero();
        JB.block<4,4>(10,3) = JRQQ.leftCols<4>();
    }

    // jacobian of (leftcam2world_t, leftcam2world_q, rightcam2world_t, rightcam2world_q) -> (leftcam2world_t, dir_left, rightcam2world_t, dir_right)

    Eigen::Matrix<double, 12, 14> JA;

    {
        Eigen::Matrix<double, 3, 7> JLQV;
        Eigen::Matrix<double, 3, 7> JRQV;

        SLAMMath::computeQuaternionVectorProduct(
            SLAMMath::convert(leftcam_to_world.unit_quaternion()),
            normalized_left,
            nullptr, &JLQV);

        SLAMMath::computeQuaternionVectorProduct(
            SLAMMath::convert(rightcam_to_world.unit_quaternion()),
            normalized_right,
            nullptr, &JRQV);

        JA.setZero();

        JA.block<3,3>(0,0).setIdentity();

        JA.block<3,4>(3,3) = JLQV.leftCols<4>();

        JA.block<3,3>(6,7).setIdentity();

        JA.block<3,4>(9,3) = JRQV.rightCols<4>();
    }

    // jacobian of (leftcam2world_t, dir_left, rightcam2world_t, dir_right) -> (mappoint_position,)

    Eigen::Matrix<double, 3, 12> J_tri;

    computeJacobianOfTriangulation(
        leftcam_to_world.translation(),
        dir_left,
        rightcam_to_world.translation(),
        dir_right,
        J_tri);

    // jacobian of (rig2world_t, rig2world_q) -> (mappoint_position, rig2world_t, rig2world_q)

    Eigen::Matrix<double, 10, 7> J;

    J.topRows<3>() = J_tri * JA * JB;
    J.bottomRows<7>().setIdentity();

    const Eigen::Matrix<double, 10, 10> sigma = J * frame->pose_covariance * J.transpose();

    cov = sigma.topRows<3>();
}
*/


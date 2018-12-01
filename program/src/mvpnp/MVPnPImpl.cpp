#include <Eigen/Eigen>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <random>
#include <iostream>
#include "MVPnPImpl.h"

MVPnP::SolverImpl::SolverImpl()
{
    mPrintError = true;
    mLMFirstLambda = 1.0;
    mLMFactor = 1.75;
    mMaxNumberOfIterations = 1000;
}

MVPnP::SolverImpl::~SolverImpl()
{
}

void MVPnP::SolverImpl::applyIncrement(Sophus::SE3d& world_to_rig, const IncrementType& increment)
{
    Eigen::Quaterniond q = world_to_rig.unit_quaternion();
    q.vec() += increment.segment<3>(3);
    q.w() += increment(6);
    q.normalize();

    world_to_rig.translation() += increment.head<3>();
    world_to_rig.setQuaternion(q);
}

double MVPnP::SolverImpl::computeErrorResidualsAndJacobianOfF( const Sophus::SE3d& world_to_rig, Eigen::Matrix<double, Eigen::Dynamic, 7>& JF, Eigen::VectorXd& residuals )
{
    double error = 0.0;

    JF.resize(2*mTotalNumberOfPoints, 7);
    residuals.resize(2*mTotalNumberOfPoints);

    int count = 0;

    for( const MVPnP::View& v : *mViews )
    {
        const Sophus::SE3d world_to_camera = v.rig_to_camera * world_to_rig;

        std::vector<cv::Point3f> points_in_camera_frame(v.points.size());

        for(int i=0; i<v.points.size(); i++)
        {
            Eigen::Vector3d in_world_frame;
            in_world_frame.x() = v.points[i].x;
            in_world_frame.y() = v.points[i].y;
            in_world_frame.z() = v.points[i].z;

            Eigen::Vector3d in_camera_frame = world_to_camera * in_world_frame;

            points_in_camera_frame[i].x = in_camera_frame.x();
            points_in_camera_frame[i].y = in_camera_frame.y();
            points_in_camera_frame[i].z = in_camera_frame.z();
        }

        std::vector<cv::Point2f> projections;
        cv::Mat jacobian;

        if( v.points.empty() == false )
        {
            cv::projectPoints(
                points_in_camera_frame,
                cv::Mat::zeros(3, 1, CV_64F),
                cv::Mat::zeros(3, 1, CV_64F),
                v.calibration_matrix,
                v.distortion_coefficients,
                projections,
                jacobian);
        }

        for( int i=0; i<v.points.size(); i++ )
        {
            Eigen::Vector3d X_world;
            X_world.x() = v.points[i].x;
            X_world.y() = v.points[i].y;
            X_world.z() = v.points[i].z;

            const Eigen::Vector2d proj{ projections[i].x, projections[i].y };
            const Eigen::Vector2d proj_ref{ v.projections[i].x, v.projections[i].y };
            const Eigen::Vector2d delta_proj = proj - proj_ref;

            // The jacobian of the function (X_world) => (x_projected).
            Eigen::Matrix<double, 2, 3> J1;
            cv::cv2eigen( jacobian(cv::Range(2*i+0, 2*i+2), cv::Range(3, 6)), J1 );

            Eigen::Matrix<double, 3, 9> MX;
            MX.setZero();
            MX.block<1,3>(0,0) = X_world.transpose();
            MX.block<1,3>(1,3) = X_world.transpose();
            MX.block<1,3>(2,6) = X_world.transpose();

            // The jacobian of the function (world2rig_q) => (world2rig_R) where world2rig_R is the vector containing the coefficients of corresponding rotation matrix.
            Eigen::Matrix<double, 9, 4> JQ2R;
            JQ2R.setZero();
            {
                const double qi = world_to_rig.unit_quaternion().x();
                const double qj = world_to_rig.unit_quaternion().y();
                const double qk = world_to_rig.unit_quaternion().z();
                const double qr = world_to_rig.unit_quaternion().w();
                //const double s = 1.0/(qi*qi + qj*qj + qk*qk + qw*qw);

                JQ2R( 0, 0 ) = 4*qi*(pow(qj, 2) + pow(qk, 2))/pow(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2), 2);
                JQ2R( 0, 1 ) = 4*qj*(pow(qj, 2) + pow(qk, 2))/pow(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2), 2) - 4*qj/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2));
                JQ2R( 0, 2 ) = 4*qk*(pow(qj, 2) + pow(qk, 2))/pow(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2), 2) - 4*qk/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2));
                JQ2R( 0, 3 ) = 4*qr*(pow(qj, 2) + pow(qk, 2))/pow(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2), 2);
                JQ2R( 1, 0 ) = -4*qi*(qi*qj - qk*qr)/pow(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2), 2) + 2*qj/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2));
                JQ2R( 1, 1 ) = 2*qi/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2)) - 4*qj*(qi*qj - qk*qr)/pow(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2), 2);
                JQ2R( 1, 2 ) = -4*qk*(qi*qj - qk*qr)/pow(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2), 2) - 2*qr/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2));
                JQ2R( 1, 3 ) = -2*qk/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2)) - 4*qr*(qi*qj - qk*qr)/pow(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2), 2);
                JQ2R( 2, 0 ) = -4*qi*(qi*qk + qj*qr)/pow(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2), 2) + 2*qk/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2));
                JQ2R( 2, 1 ) = -4*qj*(qi*qk + qj*qr)/pow(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2), 2) + 2*qr/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2));
                JQ2R( 2, 2 ) = 2*qi/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2)) - 4*qk*(qi*qk + qj*qr)/pow(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2), 2);
                JQ2R( 2, 3 ) = 2*qj/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2)) - 4*qr*(qi*qk + qj*qr)/pow(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2), 2);
                JQ2R( 3, 0 ) = -4*qi*(qi*qj + qk*qr)/pow(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2), 2) + 2*qj/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2));
                JQ2R( 3, 1 ) = 2*qi/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2)) - 4*qj*(qi*qj + qk*qr)/pow(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2), 2);
                JQ2R( 3, 2 ) = -4*qk*(qi*qj + qk*qr)/pow(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2), 2) + 2*qr/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2));
                JQ2R( 3, 3 ) = 2*qk/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2)) - 4*qr*(qi*qj + qk*qr)/pow(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2), 2);
                JQ2R( 4, 0 ) = 4*qi*(pow(qi, 2) + pow(qk, 2))/pow(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2), 2) - 4*qi/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2));
                JQ2R( 4, 1 ) = 4*qj*(pow(qi, 2) + pow(qk, 2))/pow(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2), 2);
                JQ2R( 4, 2 ) = 4*qk*(pow(qi, 2) + pow(qk, 2))/pow(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2), 2) - 4*qk/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2));
                JQ2R( 4, 3 ) = 4*qr*(pow(qi, 2) + pow(qk, 2))/pow(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2), 2);
                JQ2R( 5, 0 ) = -4*qi*(-qi*qr + qj*qk)/pow(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2), 2) - 2*qr/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2));
                JQ2R( 5, 1 ) = -4*qj*(-qi*qr + qj*qk)/pow(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2), 2) + 2*qk/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2));
                JQ2R( 5, 2 ) = 2*qj/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2)) - 4*qk*(-qi*qr + qj*qk)/pow(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2), 2);
                JQ2R( 5, 3 ) = -2*qi/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2)) - 4*qr*(-qi*qr + qj*qk)/pow(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2), 2);
                JQ2R( 6, 0 ) = -4*qi*(qi*qk - qj*qr)/pow(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2), 2) + 2*qk/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2));
                JQ2R( 6, 1 ) = -4*qj*(qi*qk - qj*qr)/pow(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2), 2) - 2*qr/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2));
                JQ2R( 6, 2 ) = 2*qi/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2)) - 4*qk*(qi*qk - qj*qr)/pow(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2), 2);
                JQ2R( 6, 3 ) = -2*qj/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2)) - 4*qr*(qi*qk - qj*qr)/pow(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2), 2);
                JQ2R( 7, 0 ) = -4*qi*(qi*qr + qj*qk)/pow(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2), 2) + 2*qr/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2));
                JQ2R( 7, 1 ) = -4*qj*(qi*qr + qj*qk)/pow(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2), 2) + 2*qk/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2));
                JQ2R( 7, 2 ) = 2*qj/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2)) - 4*qk*(qi*qr + qj*qk)/pow(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2), 2);
                JQ2R( 7, 3 ) = 2*qi/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2)) - 4*qr*(qi*qr + qj*qk)/pow(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2), 2);
                JQ2R( 8, 0 ) = 4*qi*(pow(qi, 2) + pow(qj, 2))/pow(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2), 2) - 4*qi/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2));
                JQ2R( 8, 1 ) = 4*qj*(pow(qi, 2) + pow(qj, 2))/pow(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2), 2) - 4*qj/(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2));
                JQ2R( 8, 2 ) = 4*qk*(pow(qi, 2) + pow(qj, 2))/pow(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2), 2);
                JQ2R( 8, 3 ) = 4*qr*(pow(qi, 2) + pow(qj, 2))/pow(pow(qi, 2) + pow(qj, 2) + pow(qk, 2) + pow(qr, 2), 2);
            }

            // The jacobian of the function (world2rig_t, world2rig_q) => (X_world).
            Eigen::Matrix<double, 3, 7> J2;
            J2.setZero();
            J2.leftCols<3>() = v.rig_to_camera.rotationMatrix();
            J2.rightCols<4>() = v.rig_to_camera.rotationMatrix() * MX * JQ2R;

            JF.block<2,7>(2*count, 0) = J1 * J2;
            residuals.segment<2>(2*count) = delta_proj;

            // For computing the gradient of the error, we would do:

            //gradient += 2.0 * delta_proj.transpose() * J1 * J2;

            /*
                (t_world2rig, q_world2rig) -> X_rig -> X_camera

                X_camera = t_rig2camera + R_rig2camera * ( t_world2rig + R_world2rig * X_world )

                d_X_camera = R_rig2camera * ( d_t_world2rig + d_R_world2rig * X_world )
                d_X_camera = R_rig2camera * ( d_t_world2rig + M_X_word * d_M_R_world2rig )
                d_X_camera = R_rig2camera * d_t_world2rig + R_rig2camera * M_X_word * d_M_R_world2rig
            */

            error += delta_proj.squaredNorm();

            count++;
        }
    }

    if(count != mTotalNumberOfPoints) throw std::runtime_error("internal error");

    return std::sqrt(error / double(count));
}

bool MVPnP::SolverImpl::run( const std::vector<View>& views, Sophus::SE3d& rig_to_world, std::vector< std::vector<bool> >& inliers)
{
    bool ret = true;

    if( ret )
    {
        mViews = &views;

        mTotalNumberOfPoints = 0;

        for( const View& v : views )
        {
            mTotalNumberOfPoints += v.points.size();
        }

        ret = ( mTotalNumberOfPoints > 10 );
    }

    if( ret )
    {
        Sophus::SE3d world_to_rig = rig_to_world.inverse();
        double lambda = mLMFirstLambda;
        double error = 0.0;
        Eigen::Matrix<double, Eigen::Dynamic, 7> JF;
        Eigen::VectorXd residuals;
        bool go_on = true;
        int num_iterations = 0;

        error = computeErrorResidualsAndJacobianOfF(world_to_rig, JF, residuals);

        printError(error);

        while(go_on)
        {
            Eigen::Matrix<double, 7, 7> A = JF.transpose() * JF + lambda * Eigen::Matrix<double, 7, 7>::Identity();

            Eigen::Matrix<double, 7, 1> b = -( JF.transpose() * residuals);

            Eigen::LDLT< Eigen::Matrix<double, 7, 7> > solver;
            solver.compute(A);

            Eigen::Matrix<double, 7, 1> increment = solver.solve(b);

            if( (A*increment).isApprox(b, 1.0e-6) == false )
            {
                std::cout << "Singular matrix!" << std::endl;
                go_on = false;
                ret = false;
            }
            else if( increment.norm() < 1.0e-7 )
            {
                ret = true;
                go_on = false;
            }
            else
            {
                Sophus::SE3d candidate_world_to_rig = world_to_rig;
                applyIncrement(candidate_world_to_rig, increment);

                Eigen::Matrix<double, Eigen::Dynamic, 7> candidate_JF;
                Eigen::VectorXd candidate_residuals;
                const double candidate_error = computeErrorResidualsAndJacobianOfF(candidate_world_to_rig, candidate_JF, candidate_residuals);

                if( candidate_error < error )
                {
                    go_on = ( (error - candidate_error) > 0.01 * error );
                    ret = true;

                    error = candidate_error;
                    JF.swap(candidate_JF);
                    residuals.swap(candidate_residuals);
                    world_to_rig = candidate_world_to_rig;
                    lambda /= mLMFactor;

                    printError(error);
                }
                else
                {
                    lambda *= mLMFactor;
                }
            }

            num_iterations++;
            if( num_iterations > mMaxNumberOfIterations )
            {
                ret = false;
                go_on = false;
            }
        }

        if(ret)
        {
            rig_to_world = world_to_rig.inverse();
        }
    }

    inliers.resize( views.size() );

    for( int i=0; i<views.size(); i++ )
    {
        inliers[i].assign(views[i].points.size(), true);
    }

    return ret;
}

void MVPnP::SolverImpl::printError(double error)
{
    if(mPrintError)
    {
        std::cout << "L2 reprojection error is: " << error << std::endl;
    }
}


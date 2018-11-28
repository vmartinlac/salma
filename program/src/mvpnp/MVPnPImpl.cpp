#include <Eigen/Eigen>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <random>
#include <iostream>
#include "MVPnPImpl.h"

MVPnP::SolverImpl::SolverImpl()
{
}

MVPnP::SolverImpl::~SolverImpl()
{
}

void MVPnP::SolverImpl::applyIncrement(const TangentType& increment)
{
    Eigen::Quaterniond q = mWorldToRig.unit_quaternion();
    q.vec() += increment.segment<3>(3);
    q.w() += increment(6);
    q.normalize();

    mWorldToRig.translation() += increment.head<3>();
    //mWorldToRig.setQuaternion(q);
}

double MVPnP::SolverImpl::computeError(TangentType& gradient)
{
    double error = 0.0;

    gradient.setZero();

    int count = 0;

    for( const MVPnP::View& v : *mViews )
    {
        const Eigen::Vector3d rig_to_camera_t = v.rig_to_camera.translation();
        const Eigen::Quaterniond rig_to_camera_q = v.rig_to_camera.unit_quaternion();
        const Eigen::Matrix3d rig_to_camera_R = rig_to_camera_q.toRotationMatrix();

        cv::Mat tvec;
        cv::Mat rvec;

        Eigen::Matrix<double, 6, 7> Q;

        // compute tvec, rvec and Q.
        {
            Eigen::Vector3d world_to_camera_t = rig_to_camera_R * mWorldToRig.translation() + rig_to_camera_t;
            Eigen::Quaterniond world_to_camera_q = rig_to_camera_q * mWorldToRig.unit_quaternion();

            Sophus::SE3d world_to_camera = v.rig_to_camera * mWorldToRig;

            Eigen::Matrix<double, 3, 4> L;
            Eigen::Vector3d world_to_camera_rodrigues = quaternionToRodrigues(world_to_camera_q, L);
            //Eigen::Vector3d world_to_camera_rodrigues = world_to_camera.so3().log();

            cv::eigen2cv(world_to_camera_rodrigues, rvec);
            cv::eigen2cv(world_to_camera_t, tvec);


            Eigen::Matrix4d M = Eigen::Matrix4d::Zero();

            // qx = aw*bx + bw*ax + ay*bz - az*by
            M(0,0) = rig_to_camera_q.w();
            M(0,1) = -rig_to_camera_q.z();
            M(0,2) = rig_to_camera_q.y();
            M(0,3) = rig_to_camera_q.x();

            // qy = aw*by + bw*ay + az*bx - ax*bz
            M(1,0) = rig_to_camera_q.z();
            M(1,1) = rig_to_camera_q.w();
            M(1,2) = -rig_to_camera_q.x();
            M(1,3) = rig_to_camera_q.y();

            // qz = aw*bz + bw*az + ax*by - ay*bx
            M(2,0) = -rig_to_camera_q.y();
            M(2,1) = rig_to_camera_q.x();
            M(2,2) = rig_to_camera_q.w();
            M(2,3) = rig_to_camera_q.z();

            // qw = aw*bw - ( ax*bx + ay*by + az*bz )
            M(3,0) = -rig_to_camera_q.x();
            M(3,1) = -rig_to_camera_q.y();
            M(3,2) = -rig_to_camera_q.z();
            M(3,3) = rig_to_camera_q.w();

            Q.setZero();
            Q.block<3,3>(0,0) = rig_to_camera_R;
            Q.block<3,4>(3,3) = L * M;
        }

        std::vector<cv::Point2f> projections;
        cv::Mat jac_cv;

        cv::projectPoints(v.points, rvec, tvec, v.calibration_matrix, v.distortion_coefficients, projections, jac_cv);

        Eigen::MatrixXd jac;
        cv::cv2eigen(jac_cv, jac);

        for( int i=0; i<v.points.size(); i++ )
        {
            const Eigen::Vector2d proj{ projections[i].x, projections[i].y };
            const Eigen::Vector2d proj_ref{ v.projections[i].x, v.projections[i].y };
            const Eigen::Vector2d delta_proj = proj - proj_ref;

            const Eigen::Matrix<double, 1, 7> grad = 2.0 * delta_proj.transpose() * jac.block<2,6>(2*i,0) * Q;

            gradient += 2.0 * delta_proj.transpose() * jac.block<2,6>(2*i,0) * Q;

            error += delta_proj.squaredNorm();

            count++;
        }
    }

    // divide the error and the gradient by the number of points.

    error /= double(count);
    gradient /= double(count);

    std::cout << "err = " << error << std::endl;
    //std::cout << "grad = " << gradient.transpose() << std::endl;

    return error;
}

bool MVPnP::SolverImpl::run( const std::vector<View>& views, Sophus::SE3d& rig_to_world, std::vector< std::vector<bool> >& inliers)
{
    ///////
    /*
    {
        Eigen::Matrix<double, 3, 4> J0;
        Eigen::Matrix<double, 3, 4> JN;

        Eigen::Quaterniond q(Eigen::AngleAxisd(0.3*M_PI, (Eigen::Vector3d::UnitY()+Eigen::Vector3d::UnitZ() ).normalized() ));
        q.coeffs() *= -10.0;

        const Eigen::Vector3d r0 = quaternionToRodrigues(q, J0);

        const double eps = 5.0e-3;
        for(int i=0; i<4; i++)
        {
            Eigen::Quaterniond tmp = q;
            tmp.coeffs()(i) += eps;

            Eigen::Matrix<double, 3, 4> J;
            const Eigen::Vector3d r = quaternionToRodrigues(tmp, J);
            JN.col(i) = (r - r0)*(1.0/eps);
        }

        std::cout << JN << std::endl;
        std::cout << J0 << std::endl;

        exit(0);
    }
    */
    ///////

    mViews = &views;
    mWorldToRig = rig_to_world.inverse();

    std::normal_distribution<double> normal(0.0, 1.0);
    std::default_random_engine engine;

    TangentType gradient;
    double prev = computeError(gradient);
    Sophus::SE3d good_world_to_rig = mWorldToRig;
    double step = 1.0e-4;

    for(int i=0; i<20000; i++)
    {
        applyIncrement(-step * gradient);
        double curr = computeError(gradient);
        if( curr < prev )
        {
            prev = curr;
            good_world_to_rig = mWorldToRig;
            step *= 1.5;
        }
        else
        {
            mWorldToRig = good_world_to_rig;
            step /= 1.5;
        }
    }
    std::cout << prev << std::endl;
    std::cout << mWorldToRig.inverse().translation() << std::endl;

    //const int status = lbfgs(7, x, &fx, evaluateProc, progressProc, this, &params);

    /*
    if( status == LBFGS_SUCCESS )
    {
        Eigen::Vector3d world_to_rig_t;
        world_to_rig_t.x() = x[0];
        world_to_rig_t.y() = x[1];
        world_to_rig_t.z() = x[2];

        Eigen::Quaterniond world_to_rig_q;
        world_to_rig_q.x() = x[3];
        world_to_rig_q.y() = x[4];
        world_to_rig_q.z() = x[5];
        world_to_rig_q.w() = x[6];

        Sophus::SE3d world_to_rig;
        world_to_rig.translation() = world_to_rig_t;
        world_to_rig.setQuaternion(world_to_rig_q);

        rig_to_world = world_to_rig.inverse();
    }
    */

    rig_to_world = mWorldToRig.inverse();

    inliers.resize( views.size() );

    for( int i=0; i<views.size(); i++ )
    {
        inliers[i].assign(views[i].points.size(), true);
    }

    return true;
}

Eigen::Vector3d MVPnP::SolverImpl::quaternionToRodrigues(const Eigen::Quaterniond& q, Eigen::Matrix<double, 3, 4>& J)
{
    const double N = q.vec().norm();
    Eigen::Vector3d ret;

    if( N < 1.0e-8 )
    {
        ret = q.vec();
        J <<
            1.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0;
    }
    else
    {
        double theta = 0.0;

        Eigen::Matrix<double, 1, 4> ew;
        ew << 0.0, 0.0, 0.0, 1.0;

        const double alpha = 2.0 / ( N*N + q.w()*q.w() );

        const Eigen::Matrix<double, 1, 4> J_N = Eigen::Vector4d::Constant(1.0/N);

        Eigen::Matrix<double, 1, 4> J_theta;

        if( N > std::fabs(q.w()) )
        {
            theta = M_PI - 2.0*std::atan(q.w() / N);
            J_theta = -alpha*( N*ew - q.w() * J_N);
        }
        else
        {
            theta = 2.0*std::atan(N / q.w());
            J_theta = alpha*( q.w()*J_N - N*ew );
        }

        const double theta_over_N = theta/N;

        ret.x() = q.x() * theta_over_N;
        ret.y() = q.y() * theta_over_N;
        ret.z() = q.z() * theta_over_N;

        J.setZero();
        J.leftCols<3>() += Eigen::Matrix3d::Identity() * theta_over_N;
        J += (1.0/N) * q.vec() * J_theta;
        J -= (theta/(N*N)) * q.vec() * J_N;
    }

    return ret;
}

MVPnP::Solver* MVPnP::Solver::create()
{
    return new SolverImpl();
}


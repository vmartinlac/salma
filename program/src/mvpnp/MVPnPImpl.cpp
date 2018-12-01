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
    mWorldToRig.setQuaternion(q);
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
            Q.block<3,4>(0,3) = L * M;
            Q.block<3,3>(3,0) = rig_to_camera_R;
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

    //std::cout << "err = " << error << std::endl;
    //std::cout << "grad = " << gradient.transpose() << std::endl;

    return error;
}

bool MVPnP::SolverImpl::run( const std::vector<View>& views, Sophus::SE3d& rig_to_world, std::vector< std::vector<bool> >& inliers)
{
    /*
    ///////
    std::default_random_engine engine;
    std::normal_distribution<double> normal(0.0, 1.0);
    for(int i=0; i<10; i++)
    {
        Eigen::Matrix<double, 3, 4> J0;
        Eigen::Matrix<double, 3, 4> JN;

        Eigen::Quaterniond q;
        q.x() = normal(engine);
        q.y() = normal(engine);
        q.z() = normal(engine);
        q.w() = normal(engine);
        q.normalize();

        const Eigen::Vector3d r0 = quaternionToRodrigues(q, J0);

        const double eps = 1.0e-5;
        for(int i=0; i<4; i++)
        {
            Eigen::Quaterniond tmp = q;
            tmp.coeffs()(i) += eps;

            Eigen::Matrix<double, 3, 4> J;
            const Eigen::Vector3d r = quaternionToRodrigues(tmp, J);
            JN.col(i) = (r - r0)*(1.0/eps);
        }

        std::cout << JN-J0 << std::endl;
        std::cout << J0 << std::endl;
        std::cout << std::endl;

    }
    exit(0);
    ///////
    */

    mViews = &views;
    mWorldToRig = rig_to_world.inverse();

    TangentType gradient;
    const double step = 1.0e-7;

    for(int i=0; i<35; i++)
    {
        const double err = computeError(gradient);
        std::cout << "Least square reprojection error is " << err << std::endl;

        applyIncrement(-step * gradient);
    }

    rig_to_world = mWorldToRig.inverse();

    std::cout << rig_to_world.translation() << std::endl;

    inliers.resize( views.size() );

    for( int i=0; i<views.size(); i++ )
    {
        inliers[i].assign(views[i].points.size(), true);
    }

    return false;
}

Eigen::Vector3d MVPnP::SolverImpl::quaternionToRodrigues(const Eigen::Quaterniond& q, Eigen::Matrix<double, 3, 4>& J)
{
    const double N = q.vec().norm();
    const Eigen::Vector3d ret = Sophus::SO3d(q).log();
    Eigen::Vector3d ret2;

    if( N < 1.0e-8 )
    {
        ret2 = q.vec();
        J <<
            2.0, 0.0, 0.0, 0.0,
            0.0, 2.0, 0.0, 0.0,
            0.0, 0.0, 2.0, 0.0;
        //std::cout << "A" << std::endl;
    }
    else
    {
        double theta = 0.0;

        Eigen::Matrix<double, 1, 4> ew;
        ew << 0.0, 0.0, 0.0, 1.0;

        const double alpha = 2.0 / ( N*N + q.w()*q.w() );

        Eigen::Matrix<double, 1, 4> J_N;
        J_N.head<3>() = q.vec()/N;
        J_N(3) = 0.0;

        Eigen::Matrix<double, 1, 4> J_theta;

        if( N > std::fabs(q.w()) )
        {
            //std::cout << "A" << std::endl;
            theta = M_PI - 2.0*std::atan(q.w() / N);
            J_theta = -alpha*( N*ew - q.w() * J_N);
        }
        else
        {
            //std::cout << "B" << std::endl;
            theta = 2.0*std::atan(N / q.w());
            J_theta = alpha*( q.w()*J_N - N*ew );
        }

        const double theta_over_N = theta/N;

        ret2 = q.vec() * theta_over_N;

        Eigen::Matrix<double, 3, 4> tmp;
        tmp.setZero();
        tmp.leftCols<3>() = Eigen::Matrix3d::Identity();

        J = theta_over_N * tmp + (1.0/N) * q.vec() * J_theta - (theta/(N*N)) * q.vec() * J_N;

        /*
        J.setZero();
        J.leftCols<3>() += Eigen::Matrix3d::Identity() * theta_over_N;
        J += (1.0/N) * q.vec() * J_theta;
        J -= (theta/(N*N)) * q.vec() * J_N;
        */
    }

    //std::cout << ret2.transpose() << std::endl;
    //std::cout << ret.transpose() << std::endl;

    return ret;
}


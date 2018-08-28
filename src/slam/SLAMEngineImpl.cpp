#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <iostream>
#include <QTime>
#include <QDebug>
#include "SLAMEngineImpl.h"
#include "Tracker.h"
#include "FinitePriorityQueue.h"

/*
static void correct_pose_wrt_previous(
    const Eigen::Vector3d& last_known_position,
    const Eigen::Quaterniond& last_known_attitude,
    const Eigen::Vector3d& new_position,
    const Eigen::Quaterniond& new_attitude,
    Eigen::Vector3d& translation,
    Eigen::Quaterniond& rotation)
{
    // correct attitude.

    Eigen::Quaterniond ambiguity[4];
    ambiguity[0] = Eigen::Quaterniond(Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ()));
    ambiguity[1] = Eigen::Quaterniond(Eigen::AngleAxisd(0.5*M_PI, Eigen::Vector3d::UnitZ()));
    ambiguity[2] = Eigen::Quaterniond(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()));
    ambiguity[3] = Eigen::Quaterniond(Eigen::AngleAxisd(1.5*M_PI, Eigen::Vector3d::UnitZ()));

    int j = 0;
    double best_angle = 0.0;
    for(int i=0; i<4; i++)
    {
        const double angle = last_known_attitude.angularDistance( ambiguity[i] * new_attitude );
        if( i == 0 || angle < best_angle )
        {
            j = i;
            best_angle = angle;
        }
    }
    std::cout << std::endl;

    rotation = ambiguity[j];

    // correct 

    translation.setZero();
}
*/


SLAMEngineImpl::SLAMEngineImpl()
{
}

void SLAMEngineImpl::run()
{
    std::cout << "=====================" << std::endl;
    std::cout << " SLAM ENGINE STARTED " << std::endl;
    std::cout << "=====================" << std::endl;

    setup();

    m_camera->open();

    while( isInterruptionRequested() == false )
    {
        m_camera->read(m_current_image);

        if(m_current_image.isValid())
        {
            std::cout << "-> Processing frame " << m_frame_id << std::endl;

            /* TODO: do this in a clean and more efficient way.
            cv::Mat tmp;
            cv::undistort(m_current_image.refFrame(), tmp, m_calibration_matrix, m_distortion_coefficients);
            m_current_image.refFrame() = std::move(tmp);
            */

            switch(m_mode)
            {
            case MODE_INIT:
                std::cout << "Mode is INIT" << std::endl;
                processImageInit();
                break;
            case MODE_SLAM:
                std::cout << "Mode is SLAM" << std::endl;
                processImageSLAM();
                break;
            case MODE_DEAD:
            default:
                std::cout << "Mode is DEAD" << std::endl;
                processImageDead();
                break;
            }

            writeOutput();

            // we assume that this variable is not used in INIT mode, so we do not need to set its value before this point.
            m_time_last_frame = m_current_image.getTimestamp();
            m_frame_id++;
        }
        else
        {
            QThread::msleep(5);
            //QThread::yieldCurrentThread();
        }
    }

    m_camera->close();

    std::cout << "SLAM ENGINE ENDED" << std::endl;
}

void SLAMEngineImpl::setup()
{
    // check that we have a camera.

    if( !m_camera ) throw std::runtime_error("No camera was set.");

    // setup calibration matrix.

    cv::Mat_<float> K(3,3);
    K <<
        m_parameters.fx, 0.0, m_parameters.cx,
        0.0, m_parameters.fy, m_parameters.cy,
        0.0, 0.0, 1.0;

    // setup lens distortion coefficients.

    cv::Mat_<float> lens_distortion(5, 1);
    lens_distortion <<
        m_parameters.distortion_k1,
        m_parameters.distortion_k2,
        m_parameters.distortion_p1,
        m_parameters.distortion_p2,
        m_parameters.distortion_k3;

    m_calibration_matrix = K;
    m_distortion_coefficients = lens_distortion;

    m_mode = MODE_INIT;
    m_current_image.setValid(false);
    m_time_last_frame = 0.0;
    m_frame_id = 0;

    m_camera_state.position.setZero();
    m_camera_state.attitude.setIdentity();
    m_camera_state.linear_velocity.setZero();
    m_camera_state.angular_velocity.setZero();
    m_landmarks.clear();
    m_state_covariance.resize(0, 0);
    m_candidate_landmarks.clear();

    m_tracker.setUnitLength( m_parameters.initialization_target_scale );
}

void SLAMEngineImpl::processImageInit()
{
    bool ok;
    cv::Mat pnp_rodrigues;
    cv::Mat pnp_translation;
    
    ok = m_tracker.track( m_current_image.refFrame(), true );

    if( ok )
    {
        ok = ( m_tracker.objectPoints().size() >= m_parameters.min_init_landmarks );
    }

    if( ok )
    {
        ok = cv::solvePnP(
            m_tracker.objectPoints(),
            m_tracker.imagePoints(),
            m_calibration_matrix,
            m_distortion_coefficients,
            pnp_rodrigues,
            pnp_translation,
            false,
            cv::SOLVEPNP_ITERATIVE );
    }

    if( ok )
    {
        // copy camera pose data.

        Eigen::Vector3d rodrigues;
        rodrigues <<
            pnp_rodrigues.at<float>(0, 0),
            pnp_rodrigues.at<float>(1, 0),
            pnp_rodrigues.at<float>(2, 0);

        Eigen::Vector3d world_to_camera_translation;
        world_to_camera_translation <<
            pnp_translation.at<float>(0,0),
            pnp_translation.at<float>(1,0),
            pnp_translation.at<float>(2,0);

        const double norm = rodrigues.norm();

        Eigen::Quaterniond world_to_camera_rotation;
        if( norm > 1.0e-9 )
        {
            world_to_camera_rotation.vec() = sin(0.5*norm) * rodrigues / norm;
            world_to_camera_rotation.w() = cos(0.5*norm);
        }
        else
        {
            world_to_camera_rotation.setIdentity();
        }

        m_camera_state.attitude = world_to_camera_rotation.inverse();
        m_camera_state.position = -( m_camera_state.attitude * world_to_camera_translation );
        m_camera_state.linear_velocity.setZero();
        m_camera_state.angular_velocity.setZero();
    }

    if(ok)
    {
        const std::vector<cv::Point3f>& object_points = m_tracker.objectPoints();
        const std::vector<cv::Point2f>& image_points = m_tracker.imagePoints();

        const int N = object_points.size();

        m_landmarks.clear();
        m_landmarks.reserve( N );

        cv::Rect viewport(
            m_parameters.patch_size,
            m_parameters.patch_size,
            m_current_image.refFrame().cols - 2*m_parameters.patch_size,
            m_current_image.refFrame().rows - 2*m_parameters.patch_size );

        for(int i=0; i<N; i++)
        {
            if( viewport.contains( image_points[i] ) )
            {
                m_landmarks.emplace_back();
                Landmark& lm = m_landmarks.back();

                cv::getRectSubPix(
                    m_current_image.refFrame(),
                    cv::Size( m_parameters.patch_size, m_parameters.patch_size ),
                    image_points[i],
                    lm.patch );

                lm.position.x() = object_points[i].x;
                lm.position.y() = object_points[i].y;
                lm.position.z() = object_points[i].z;
                lm.num_failed_detections = 0;
                lm.num_successful_detections = 1;
                lm.last_seen_frame = m_frame_id;
            }
        }

        const int M = m_landmarks.size();
        if( M >= m_parameters.min_init_landmarks )
        {
            m_state_covariance.resize( 13 + 3*M, 13 + 3*M );

            m_state_covariance.setZero();

            double mean_sigma_landmark = 0.0;

            for(int i=0; i<M; i++)
            {
                Eigen::Vector3d in_camera_frame = m_camera_state.attitude.inverse() * (m_landmarks[i].position - m_camera_state.position);
                const double depth = std::max( m_parameters.min_distance_to_camera, in_camera_frame.z() );
                const double eps = double(m_current_image.width()) * 0.9 / 640.0;
                const double sigma_landmark = eps * depth / std::max(m_parameters.fx, m_parameters.fy);
                m_state_covariance.diagonal().segment(13+3*i, 3).fill(sigma_landmark*sigma_landmark);

                mean_sigma_landmark += sigma_landmark;
            }

            mean_sigma_landmark /= double(M);

            const double sigma_position = 0.8 * mean_sigma_landmark;
            const double sigma_attitude = 0.015; // see misc/initial_state_variance.py.
            const double sigma_linear_velocity = 0.0;
            const double sigma_angular_velocity = 0.0;
            m_state_covariance.diagonal().segment<3>(0).fill(sigma_position*sigma_position);
            m_state_covariance.diagonal().segment<4>(3).fill(sigma_attitude*sigma_attitude);
            m_state_covariance.diagonal().segment<3>(7).fill(sigma_linear_velocity*sigma_linear_velocity);
            m_state_covariance.diagonal().segment<3>(10).fill(sigma_angular_velocity*sigma_angular_velocity);

            m_mode = MODE_SLAM;

            qInfo() << "Successful initialization.";
            qInfo() << "Switching to SLAM mode.";
        }
        else
        {
            m_landmarks.clear();
        }
    }
}

/*
void SLAMEngineImpl::processImageTT()
{
    ;
}
*/

void SLAMEngineImpl::processImageSLAM()
{
    QTime chrono;
    int time_prediction = 0;
    int time_update = 0;

    Eigen::VectorXd state_mu;
    Eigen::MatrixXd state_sigma;

    chrono.start();
    EKFPredict(state_mu, state_sigma);
    time_prediction = chrono.elapsed();

    chrono.start();
    EKFUpdate(state_mu, state_sigma);
    time_update = chrono.elapsed();

    saveState(state_mu, state_sigma);

    std::cout << "prediction : update = " << time_prediction << " : " << time_update << std::endl;

    // TODO: manage candidate landmarks.
}

void SLAMEngineImpl::EKFPredict(Eigen::VectorXd& pred_mu, Eigen::MatrixXd& pred_sigma)
{
    const double dt = m_current_image.getTimestamp() - m_time_last_frame;

    // retrieve constants and current state.

    const int num_landmarks = m_landmarks.size();
    const int dim = 13 + 3*num_landmarks;

    const double x1 = m_camera_state.position.x();
    const double x2 = m_camera_state.position.y();
    const double x3 = m_camera_state.position.z();

    const double a0 = m_camera_state.attitude.w();
    const double a1 = m_camera_state.attitude.x();
    const double a2 = m_camera_state.attitude.y();
    const double a3 = m_camera_state.attitude.z();

    const double v1 = m_camera_state.linear_velocity.x();
    const double v2 = m_camera_state.linear_velocity.y();
    const double v3 = m_camera_state.linear_velocity.z();

    double w1 = m_camera_state.angular_velocity.x();
    double w2 = m_camera_state.angular_velocity.y();
    double w3 = m_camera_state.angular_velocity.z();

    double norm_w = std::sqrt(w1*w1 + w2*w2 + w3*w3);
    double axis1;
    double axis2;
    double axis3;
    const double theta = 0.5*norm_w*dt;
    const double cos_theta = std::cos(theta);
    const double sin_theta = std::sin(theta);
    double sin_theta_over_norm_w;

    if( norm_w < 1.0e-10 )
    {
        norm_w = 0.0;
        axis1 = 1.0;
        axis2 = 0.0;
        axis3 = 0.0;
        sin_theta_over_norm_w = 0.0;
    }
    else
    {
        axis1 = w1/norm_w;
        axis2 = w2/norm_w;
        axis3 = w3/norm_w;
        sin_theta_over_norm_w = sin_theta / norm_w;
    }

    const double r0 = cos_theta;
    const double r1 = sin_theta * axis1;
    const double r2 = sin_theta * axis2;
    const double r3 = sin_theta * axis3;

    // fills pred_mu vector.

    pred_mu.resize(dim);

    pred_mu.head<13>() <<
        x1 + dt*v1,
        x2 + dt*v2,
        x3 + dt*v3,
        a0*r0 - a1*r1 - a2*r2 - a3*r3,
        a0*r1 + r0*a1 + (a2*r3 - a3*r2),
        a0*r2 + r0*a2 + (a3*r1 - a1*r3),
        a0*r3 + r0*a3 + (a1*r2 - a2*r1),
        v1,
        v2,
        v3,
        w1,
        w2,
        w3;

    for(int i=0; i<num_landmarks; i++)
    {
        pred_mu.segment<3>(13+3*i) = m_landmarks[i].position;
    }

    // J is the jacobian of the function which gives new state from current state.

    Eigen::SparseMatrix<double, Eigen::RowMajor> J(dim, dim);
    J.reserve(40 + 3*num_landmarks);

    // position.

    J.insert(0,0) = 1.0;
    J.insert(0,7) = dt;
    J.insert(1,1) = 1.0;
    J.insert(1,8) = dt;
    J.insert(2,2) = 1.0;
    J.insert(2,9) = dt;

    // attitude.

    // Generated automatically by python script system2.py.
    // BEGIN
    J.insert(3,3) = r0;
    J.insert(3,4) = -r1;
    J.insert(3,5) = -r2;
    J.insert(3,6) = -r3;
    J.insert(3,10) = -0.5*a0*dt*r1 - 0.5*a1*dt*axis1*axis1*cos_theta + 1.0*a1*axis1*axis1*sin_theta_over_norm_w - a1*norm_w*sin_theta - 0.5*a2*dt*axis1*axis2*cos_theta + 1.0*a2*axis1*axis2*sin_theta_over_norm_w - 0.5*a3*dt*axis1*axis3*cos_theta + 1.0*a3*axis1*axis3*sin_theta_over_norm_w;
    J.insert(3,11) = -0.5*a0*dt*r2 - 0.5*a1*dt*axis1*axis2*cos_theta + 1.0*a1*axis1*axis2*sin_theta_over_norm_w - 0.5*a2*dt*axis2*axis2*cos_theta + 1.0*a2*axis2*axis2*sin_theta_over_norm_w - a2*norm_w*sin_theta - 0.5*a3*dt*axis2*axis3*cos_theta + 1.0*a3*axis2*axis3*sin_theta_over_norm_w;
    J.insert(3,12) = -0.5*a0*dt*r3 - 0.5*a1*dt*axis1*axis3*cos_theta + 1.0*a1*axis1*axis3*sin_theta_over_norm_w - 0.5*a2*dt*axis2*axis3*cos_theta + 1.0*a2*axis2*axis3*sin_theta_over_norm_w - 0.5*a3*dt*axis3*axis3*cos_theta + 1.0*a3*axis3*axis3*sin_theta_over_norm_w - a3*norm_w*sin_theta;
    J.insert(4,3) = r1;
    J.insert(4,4) = r0;
    J.insert(4,5) = r3;
    J.insert(4,6) = -r2;
    J.insert(4,10) = 0.5*a0*dt*axis1*axis1*cos_theta - 1.0*a0*axis1*axis1*sin_theta_over_norm_w + a0*norm_w*sin_theta - 0.5*a1*dt*r1 + 0.5*a2*dt*axis1*axis3*cos_theta - 1.0*a2*axis1*axis3*sin_theta_over_norm_w - 0.5*a3*dt*axis1*axis2*cos_theta + 1.0*a3*axis1*axis2*sin_theta_over_norm_w;
    J.insert(4,11) = 0.5*a0*dt*axis1*axis2*cos_theta - 1.0*a0*axis1*axis2*sin_theta_over_norm_w - 0.5*a1*dt*r2 + 0.5*a2*dt*axis2*axis3*cos_theta - 1.0*a2*axis2*axis3*sin_theta_over_norm_w - 0.5*a3*dt*axis2*axis2*cos_theta + 1.0*a3*axis2*axis2*sin_theta_over_norm_w - a3*norm_w*sin_theta;
    J.insert(4,12) = 0.5*a0*dt*axis1*axis3*cos_theta - 1.0*a0*axis1*axis3*sin_theta_over_norm_w - 0.5*a1*dt*r3 + 0.5*a2*dt*axis3*axis3*cos_theta - 1.0*a2*axis3*axis3*sin_theta_over_norm_w + a2*norm_w*sin_theta - 0.5*a3*dt*axis2*axis3*cos_theta + 1.0*a3*axis2*axis3*sin_theta_over_norm_w;
    J.insert(5,3) = r2;
    J.insert(5,4) = -r3;
    J.insert(5,5) = r0;
    J.insert(5,6) = r1;
    J.insert(5,10) = 0.5*a0*dt*axis1*axis2*cos_theta - 1.0*a0*axis1*axis2*sin_theta_over_norm_w - 0.5*a1*dt*axis1*axis3*cos_theta + 1.0*a1*axis1*axis3*sin_theta_over_norm_w - 0.5*a2*dt*r1 + 0.5*a3*dt*axis1*axis1*cos_theta - 1.0*a3*axis1*axis1*sin_theta_over_norm_w + a3*norm_w*sin_theta;
    J.insert(5,11) = 0.5*a0*dt*axis2*axis2*cos_theta - 1.0*a0*axis2*axis2*sin_theta_over_norm_w + a0*norm_w*sin_theta - 0.5*a1*dt*axis2*axis3*cos_theta + 1.0*a1*axis2*axis3*sin_theta_over_norm_w - 0.5*a2*dt*r2 + 0.5*a3*dt*axis1*axis2*cos_theta - 1.0*a3*axis1*axis2*sin_theta_over_norm_w;
    J.insert(5,12) = 0.5*a0*dt*axis2*axis3*cos_theta - 1.0*a0*axis2*axis3*sin_theta_over_norm_w - 0.5*a1*dt*axis3*axis3*cos_theta + 1.0*a1*axis3*axis3*sin_theta_over_norm_w - a1*norm_w*sin_theta - 0.5*a2*dt*r3 + 0.5*a3*dt*axis1*axis3*cos_theta - 1.0*a3*axis1*axis3*sin_theta_over_norm_w;
    J.insert(6,3) = r3;
    J.insert(6,4) = r2;
    J.insert(6,5) = -r1;
    J.insert(6,6) = r0;
    J.insert(6,10) = 0.5*a0*dt*axis1*axis3*cos_theta - 1.0*a0*axis1*axis3*sin_theta_over_norm_w + 0.5*a1*dt*axis1*axis2*cos_theta - 1.0*a1*axis1*axis2*sin_theta_over_norm_w - 0.5*a2*dt*axis1*axis1*cos_theta + 1.0*a2*axis1*axis1*sin_theta_over_norm_w - a2*norm_w*sin_theta - 0.5*a3*dt*r1;
    J.insert(6,11) = 0.5*a0*dt*axis2*axis3*cos_theta - 1.0*a0*axis2*axis3*sin_theta_over_norm_w + 0.5*a1*dt*axis2*axis2*cos_theta - 1.0*a1*axis2*axis2*sin_theta_over_norm_w + a1*norm_w*sin_theta - 0.5*a2*dt*axis1*axis2*cos_theta + 1.0*a2*axis1*axis2*sin_theta_over_norm_w - 0.5*a3*dt*r2;
    J.insert(6,12) = 0.5*a0*dt*axis3*axis3*cos_theta - 1.0*a0*axis3*axis3*sin_theta_over_norm_w + a0*norm_w*sin_theta + 0.5*a1*dt*axis2*axis3*cos_theta - 1.0*a1*axis2*axis3*sin_theta_over_norm_w - 0.5*a2*dt*axis1*axis3*cos_theta + 1.0*a2*axis1*axis3*sin_theta_over_norm_w - 0.5*a3*dt*r3;
    // END

    // linear velocity.

    J.insert(7, 7) = 1.0;
    J.insert(8, 8) = 1.0;
    J.insert(9, 9) = 1.0;

    // angular velocity.

    J.insert(10, 10) = 1.0;
    J.insert(11, 11) = 1.0;
    J.insert(12, 12) = 1.0;

    for(int i=0; i<num_landmarks; i++)
    {
        J.insert(13+3*i+0, 13+3*i+0) = 1.0;
        J.insert(13+3*i+1, 13+3*i+1) = 1.0;
        J.insert(13+3*i+2, 13+3*i+2) = 1.0;
    }

    J.makeCompressed();

    // TODO: setup these constants correctly.
    //const double sigma_v = dt*5.3; //dt*0.1;
    //const double sigma_w = dt*1.2;
    const double sigma_v = dt*5.3; //dt*0.1;
    const double sigma_w = dt*1.2;

    Eigen::SparseMatrix<double> Q(dim, dim);
    Q.reserve(6);
    Q.insert(7,7) = sigma_v*sigma_v;
    Q.insert(8,8) = sigma_v*sigma_v;
    Q.insert(9,9) = sigma_v*sigma_v;
    Q.insert(10,10) = sigma_w*sigma_w;
    Q.insert(11,11) = sigma_w*sigma_w;
    Q.insert(12,12) = sigma_w*sigma_w;

    Q.makeCompressed();

    pred_sigma = J * (m_state_covariance + Q) * J.transpose();
}

void SLAMEngineImpl::EKFUpdate(Eigen::VectorXd& pred_mu, Eigen::MatrixXd& pred_sigma)
{
    const int num_landmarks = m_landmarks.size();
    const int dim = 13 + 3*num_landmarks;
    const cv::Rect viewport(
        m_parameters.patch_size,
        m_parameters.patch_size,
        m_current_image.refFrame().cols - 2*m_parameters.patch_size,
        m_current_image.refFrame().rows - 2*m_parameters.patch_size );

    Eigen::VectorXd residuals(2*num_landmarks);

    Eigen::SparseMatrix<double, Eigen::RowMajor> J(2*num_landmarks, dim);
    J.reserve(20*num_landmarks);

    FinitePriorityQueue<int,int> priority_queue(m_parameters.max_landmarks_per_frame);

    // retrieve of compute some constants.

    const double x1 = pred_mu(0);
    const double x2 = pred_mu(1);
    const double x3 = pred_mu(2);

    const double a0 = pred_mu(3);
    const double a1 = pred_mu(4);
    const double a2 = pred_mu(5);
    const double a3 = pred_mu(6);

    // rotation matrix from camera frame to world frame.
    const double R11 = 1.0 - 2.0*(a2*a2 + a3*a3);
    const double R12 = 2.0*(a1*a2 - a3*a0);
    const double R13 = 2.0*(a1*a3 + a2*a0);
    const double R21 = 2.0*(a1*a2 + a0*a3);
    const double R22 = 1.0 - 2.0*(a1*a1 + a3*a3);
    const double R23 = 2.0*(a2*a3 - a1*a0);
    const double R31 = 2.0*(a1*a3 - a0*a2);
    const double R32 = 2.0*(a2*a3 + a1*a0);
    const double R33 = 1.0 - 2.0*(a1*a1 + a2*a2);

    const double c1 = m_parameters.cx;
    const double c2 = m_parameters.cy;

    const double f1 = m_parameters.fx;
    const double f2 = m_parameters.fy;

    // compute find landmarks, compute residuals and jacobian.

    for(int i=0; i<num_landmarks; i++)
    {
        Landmark& lm = m_landmarks[i];

        const double y1 = lm.position.x();
        const double y2 = lm.position.y();
        const double y3 = lm.position.z();

        const double d1 = y1 - x1;
        const double d2 = y2 - x2;
        const double d3 = y3 - x3;

        const double ycam1 = R11*d1 + R21*d2 + R31*d3;
        const double ycam2 = R12*d1 + R22*d2 + R32*d3;
        const double ycam3 = R13*d1 + R23*d2 + R33*d3;

        double u1 = 0.0;
        double u2 = 0.0;

        cv::Point2f found_point(0.0, 0.0);

        bool ok = true;

        if( ok)
        {
            // TODO: check this line.
            ok = ( ycam3 > 1.0e-8 && ycam3 > m_parameters.min_distance_to_camera );
        }

        if( ok )
        {
            u1 = c1 + f1 * ycam1/ycam3;
            u2 = c2 + f2 * ycam2/ycam3;

            ok = viewport.contains( cv::Point2f(u1, u2) );
        }

        if( ok )
        {
            // Generated by python script update.py
            // BEGIN
            J.insert(2*i+0, 0) = f1*R13*ycam1/(ycam3*ycam3) + f1*(-R11)/ycam3;
            J.insert(2*i+0, 1) = f1*R23*ycam1/(ycam3*ycam3) + f1*(-R21)/ycam3;
            J.insert(2*i+0, 2) = f1*(-R31)/ycam3 + f1*R33*ycam1/(ycam3*ycam3);
            J.insert(2*i+0, 3) = f1*(2*a1*d2 - 2*a2*d1)*ycam1/(ycam3*ycam3) + f1*(-2*a2*d3 + 2*a3*d2)/ycam3;
            J.insert(2*i+0, 4) = f1*(2*a2*d2 + 2*a3*d3)/ycam3 + f1*(2*a0*d2 + 4*a1*d3 - 2*a3*d1)*ycam1/(ycam3*ycam3);
            J.insert(2*i+0, 5) = f1*(-2*a0*d1 + 4*a2*d3 - 2*a3*d2)*ycam1/(ycam3*ycam3) + f1*(-2*a0*d3 + 2*a1*d2 - 4*a2*d1)/ycam3;
            J.insert(2*i+0, 6) = f1*(-2*a1*d1 - 2*a2*d2)*ycam1/(ycam3*ycam3) + f1*(2*a0*d2 + 2*a1*d3 - 4*a3*d1)/ycam3;
            J.insert(2*i+0, 13+3*i+0) = f1*(-R13)*ycam1/(ycam3*ycam3) + f1*R11/ycam3;
            J.insert(2*i+0, 13+3*i+1) = f1*(-R23)*ycam1/(ycam3*ycam3) + f1*R21/ycam3;
            J.insert(2*i+0, 13+3*i+2) = f1*R31/ycam3 + f1*(-R33)*ycam1/(ycam3*ycam3);
            J.insert(2*i+1, 0) = f2*R13*ycam2/(ycam3*ycam3) + f2*(-R12)/ycam3;
            J.insert(2*i+1, 1) = f2*R23*ycam2/(ycam3*ycam3) + f2*(2*(a1*a1) + 2*(a3*a3) - 1)/ycam3;
            J.insert(2*i+1, 2) = f2*(-R32)/ycam3 + f2*R33*ycam2/(ycam3*ycam3);
            J.insert(2*i+1, 3) = f2*(2*a1*d2 - 2*a2*d1)*ycam2/(ycam3*ycam3) + f2*(2*a1*d3 - 2*a3*d1)/ycam3;
            J.insert(2*i+1, 4) = f2*(2*a0*d2 + 4*a1*d3 - 2*a3*d1)*ycam2/(ycam3*ycam3) + f2*(2*a0*d3 - 4*a1*d2 + 2*a2*d1)/ycam3;
            J.insert(2*i+1, 5) = f2*(2*a1*d1 + 2*a3*d3)/ycam3 + f2*(-2*a0*d1 + 4*a2*d3 - 2*a3*d2)*ycam2/(ycam3*ycam3);
            J.insert(2*i+1, 6) = f2*(-2*a1*d1 - 2*a2*d2)*ycam2/(ycam3*ycam3) + f2*(-2*a0*d1 + 2*a2*d3 - 4*a3*d2)/ycam3;
            J.insert(2*i+1, 13+3*i+0) = f2*(-R13)*ycam2/(ycam3*ycam3) + f2*R12/ycam3;
            J.insert(2*i+1, 13+3*i+1) = f2*(-R23)*ycam2/(ycam3*ycam3) + f2*R22/ycam3;
            J.insert(2*i+1, 13+3*i+2) = f2*R32/ycam3 + f2*(-R33)*ycam2/(ycam3*ycam3);
            // END

            Eigen::Matrix2d covariance = J.block(2*i,0,2,dim) * pred_sigma * J.block(2*i,0,2,dim).transpose();

            // TODO: compute a general ellipse instead of this axis aligned box.
            const double max_radius = double(m_current_image.width()) * 20.0/640.0;
            const double min_radius = double(m_current_image.width()) * 2.0/640.0;
            const double box_radius_1 = std::max(std::min(4.0 * std::sqrt( covariance(0,0) ), max_radius), min_radius);
            const double box_radius_2 = std::max(std::min(4.0 * std::sqrt( covariance(1,1) ), max_radius), min_radius);

            // TODO: remove
            //std::cerr << "B " << m_frame_id << ' ' << box_radius_1 << ' ' << box_radius_2 << std::endl;
            //

            ok = findPatch(
                m_landmarks[i].patch,
                cv::Point2f(u1, u2),
                cv::Vec2f(box_radius_1, box_radius_2),
                found_point );
        }

        if(ok)
        {
            residuals(2*i+0) = found_point.x - u1;
            residuals(2*i+1) = found_point.y - u2;
            //std::cerr << "R " << m_frame_id << ' ' << found_point.x - u1 << ' ' << found_point.y - u2 << std::endl;

            priority_queue.push(i, -m_landmarks[i].last_seen_frame);
        }
        else
        {
            residuals(2*i+0) = 0.0;
            residuals(2*i+1) = 0.0;
        }
    }

    const int num_found = priority_queue.size();

    std::cout << "Number of landmarks found in current frame: " << num_found << std::endl;

    if(num_found > 0)
    {
        // compute a matrix to extract out residuals corresponding to found landmarks.

        Eigen::SparseMatrix<double> proj(2*num_found, 2*num_landmarks);
        proj.reserve(4*num_found);

        Eigen::SparseMatrix<double> noise(2*num_found, 2*num_found);
        noise.reserve(2*num_found);

        {
            int j = 0;
            for(int i : priority_queue)
            {
                m_landmarks[i].last_seen_frame = m_frame_id;
                proj.insert(2*j+0, 2*i+0) = 1.0;
                proj.insert(2*j+1, 2*i+1) = 1.0;

                const double sigma = 2.0 * double(m_current_image.width()) / 640.0; // TODO: define this constant somewhere else.
                noise.insert(2*j+0, 2*j+0) = sigma*sigma;
                noise.insert(2*j+1, 2*j+1) = sigma*sigma;

                j++;
            }

            if( j != num_found ) throw std::logic_error("internal error");
        }

        auto projected_J = proj * J;
        auto projected_residuals = proj * residuals;

        Eigen::MatrixXd S = projected_J * pred_sigma * projected_J.transpose() + noise;

        Eigen::LDLT< Eigen::MatrixXd > solver;
        solver.compute( S );

        Eigen::VectorXd new_mu = pred_mu + pred_sigma * projected_J.transpose() * solver.solve( projected_residuals );
        Eigen::MatrixXd new_sigma = pred_sigma - pred_sigma * projected_J.transpose() * solver.solve( projected_J * pred_sigma );

        pred_mu.swap(new_mu);
        pred_sigma.swap(new_sigma);
    }
}

void SLAMEngineImpl::saveState(Eigen::VectorXd& mu, Eigen::MatrixXd& sigma)
{
    m_camera_state.position = mu.segment<3>(0);
    m_camera_state.attitude.w() = mu(3);
    m_camera_state.attitude.vec() = mu.segment<3>(4);
    m_camera_state.linear_velocity = mu.segment<3>(7);
    m_camera_state.angular_velocity = mu.segment<3>(10);

    const int num_landmarks = m_landmarks.size();

    for(int i=0; i<num_landmarks; i++)
    {
        m_landmarks[i].position = mu.segment<3>(13+3*i);
    }

    m_state_covariance.swap(sigma);
}

void SLAMEngineImpl::processImageDead()
{
    m_output->beginWrite();
    m_output->image = m_current_image.refFrame().clone();
    m_output->mode = "DEAD";
    m_output->endWrite();
    m_output->updated();
}

void SLAMEngineImpl::writeOutput()
{
    cv::Mat output_image = m_current_image.refFrame().clone();

    if( m_mode == MODE_SLAM)
    {
        for(const Landmark& lm : m_landmarks)
        {
            Eigen::Vector3d pos = m_camera_state.attitude.inverse() * (lm.position - m_camera_state.position);

            cv::Point2f pt(
                m_parameters.cx + m_parameters.fx*pos.x()/pos.z(),
                m_parameters.cy + m_parameters.fy*pos.y()/pos.z() );

            const int radius = output_image.cols*5/640;

            cv::Scalar color;
            if( lm.last_seen_frame == m_frame_id )
            {
                color = cv::Scalar(32, 255, 32);
            }
            else
            {
                color = cv::Scalar(255, 32, 32);
            }

            cv::circle(output_image, pt, radius+1, cv::Scalar(0,0,0), -1);
            cv::circle(output_image, pt, radius, color, -1);
        }

        // draw patches.

        /*
        cv::Rect image_rect( 0, 0, output_image.cols, output_image.rows );

        int k = 0;
        for(const Landmark& lm : m_landmarks)
        {
            const int s = m_parameters.patch_size;
            const int mod = output_image.cols/s;
            const int x = (k % mod) * s;
            const int y = (k / mod) * s;

            cv::Rect patch_rect(x, y, lm.patch.cols, lm.patch.rows);

            if( (patch_rect | image_rect) == image_rect )
            {
                output_image(patch_rect) = lm.patch;
            }

            k++;
        }
        */
    }

    std::vector<SLAMOutputLandmark> output_landmarks;
    output_landmarks.reserve( m_landmarks.size() );
    for(Landmark& lm : m_landmarks)
    {
        SLAMOutputLandmark olm;
        olm.position = lm.position;

        output_landmarks.push_back(olm);
    }

    m_output->beginWrite();

    switch( m_mode )
    {
    case MODE_INIT:
        m_output->mode = "INIT";
        break;
    case MODE_SLAM:
        m_output->mode = "SLAM";
        break;
    case MODE_DEAD:
        m_output->mode = "DEAD";
        break;
    default:
        throw std::runtime_error("internal error");
        break;
    }

    m_output->frame_id = m_frame_id;
    m_output->timestamp = m_current_image.getTimestamp();
    m_output->image = output_image;
    m_output->position = m_camera_state.position;
    m_output->attitude = m_camera_state.attitude;
    m_output->linear_velocity = m_camera_state.linear_velocity;
    m_output->angular_velocity = m_camera_state.angular_velocity;
    m_output->landmarks.swap(output_landmarks);

    m_output->endWrite();

    m_output->updated();
}

bool SLAMEngineImpl::findPatch(
    const cv::Mat& patch,
    const cv::Point2f& ellipse_center,
    const cv::Vec2f& ellipse_radii,
    cv::Point2f& result)
{
    int area_width = std::max<int>(
        int(std::ceil(2.0*ellipse_radii(0))) + patch.cols,
        patch.cols*2/3 );

    int area_height = std::max<int>(
        int(std::ceil(2.0*ellipse_radii(1))) + patch.rows,
        patch.rows*2/3 );

    cv::Rect area(
        int(std::floor(ellipse_center.x - ellipse_radii(0))) - area_width/2,
        int(std::floor(ellipse_center.y - ellipse_radii(1))) - area_height/2,
        area_width,
        area_height);

    cv::Rect max_area( cv::Point(0,0), m_current_image.refFrame().size() );

    bool ret = false;

    if( (area | max_area) == max_area )
    {
        cv::Mat response;
        cv::matchTemplate(
            m_current_image.refFrame()(area),
            patch,
            response,
            cv::TM_SQDIFF);

        double value;
        cv::Point loc;
        cv::minMaxLoc( response, &value, nullptr, &loc, nullptr);

        const double threshold = 5000.0 * double( patch.rows * patch.cols );

        if( value < threshold )
        {
            result = area.tl() + loc + cv::Point2i(patch.cols/2, patch.rows/2);
            cv::circle( m_current_image.refFrame(), result, 3, cv::Scalar(0,255,0), -1);
            std::cout << cv::norm(ellipse_center - result) << std::endl;

            ret = true;
        }
    }

    return ret;
}

SLAMEngine* SLAMEngine::create()
{
    return new SLAMEngineImpl();
}


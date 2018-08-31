#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <iostream>
#include <QTime>
#include <QDebug>
#include "SLAMEngineImpl.h"
#include "Tracker.h"

inline void quaternion2rodrigues(const Eigen::Quaterniond& quaternion, cv::Mat& rodrigues, bool invert, Eigen::Matrix<double, 3, 4>* jacobian=nullptr)
{
    Eigen::Quaterniond a = quaternion.normalized();

    Eigen::Vector3d r;

    const double s = a.vec().norm();

    const double eps = 1.0e-8;

    // TODO: fill the jacobian.

    if( s > eps )
    {
        if( std::fabs( a.w() ) > eps )
        {
            if( invert )
            {
                r = a.vec() * ( 2.0 * std::atan( s / a.w() ) / s );

                if( jacobian != nullptr )
                {
                    ;
                }
            }
            else
            {
                r = -a.vec() * ( 2.0 * std::atan( s / a.w() ) / s );
            }
        }
        else
        {
            if( invert )
            {
                r = a.vec() * (M_PI / s);
            }
            else
            {
                r = -a.vec() * (M_PI / s);
            }
        }
    }
    else
    {
        r.setZero();
    }

    rodrigues.create(3, 1, CV_32F);
    rodrigues.at<float>(0) = r.x();
    rodrigues.at<float>(1) = r.y();
    rodrigues.at<float>(2) = r.z();
}

SLAMEngineImpl::SLAMEngineImpl()
{
    m_detector = cv::GFTTDetector::create( 600 );
    m_descriptor = cv::ORB::create();
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

            // we assume that m_time_last_frame is not used in INIT mode, so we do not need to set its value before this point.
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

            // BEGIN compute descriptors.
            {
                /*
                std::vector<cv::KeyPoint> corners;
                cv::Mat descriptors;

                m_feature->detectAndCompute( m_current_image.refFrame(), corners, descriptors );
                TODO
                */

                std::vector<cv::KeyPoint> original_keypoints = m_tracker.imageKeyPoints();
                std::vector<cv::KeyPoint> keypoints = original_keypoints;
                cv::Mat descriptors;

                m_feature->compute(m_current_image.refFrame(), keypoints, descriptors);
                if( keypoints.size() != original_keypoints.size() ) { throw std::runtime_error("error"); }

                for(int i=0; i<M; i++)
                {
                    if( cv::norm( keypoints[i].pt - original_keypoints[i].pt ) > 2.0 ) { throw std::runtime_error("error"); }
                    m_landmarks[i].descriptor = descriptors.row(i);
                }

            }
            // END

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
    // define some constants.

    const int num_landmarks = m_landmarks.size();
    const int dim = 13 + 3*num_landmarks;

    const cv::Rect viewport(
        m_parameters.patch_size,
        m_parameters.patch_size,
        m_current_image.refFrame().cols - 2*m_parameters.patch_size,
        m_current_image.refFrame().rows - 2*m_parameters.patch_size );

    const double measurement_sigma = 2.5*m_current_image.width()/640.0;

    // define some variables.

    Eigen::VectorXd residuals(2*num_landmarks);

    Eigen::SparseMatrix<double, Eigen::RowMajor> J(2*num_landmarks, dim);

    std::vector<bool> found(num_landmarks);

    int num_found = 0;

    std::vector<int> selection;

    std::vector<cv::Point3f> to_project;

    std::vector<cv::Point2f> projected;

    cv::Mat landmark_descriptors;

    std::vector<cv::KeyPoint> corners;

    cv::Mat corner_descriptors;

    // determine which points we will attempt to find in the image.

    selection.reserve(num_landmarks);
    to_project.reserve(num_landmarks);

    for(int i=0; i<num_landmarks; i++)
    {
        Eigen::Vector3d in_camera_frame = m_camera_state.attitude.inverse() * ( m_landmarks[i].position - m_camera_state.position );

        if( in_camera_frame.z() > 1.0e-8 && in_camera_frame.z() > m_parameters.min_distance_to_camera )
        {
            in_camera_frame /= in_camera_frame.z();
            const double x = in_camera_frame.x() * m_parameters.fx + m_parameters.cx;
            const double y = in_camera_frame.y() * m_parameters.fy + m_parameters.cy;

            if( viewport.contains(cv::Point2f(x, y)) )
            {
                selection.push_back(i);

                to_project.push_back( cv::Point3f(
                    m_landmarks[i].position.x(),
                    m_landmarks[i].position.y(),
                    m_landmarks[i].position.z() ));
            }
        }
    }

    const int num_visible = to_project.size();

    // project points in the image and compute the jacobian of this operation.

    if( num_visible > 0 )
    {
        cv::Mat rodrigues;
        Eigen::Matrix<double, 3, 4> J1;

        quaternion2rodrigues( m_camera_state.attitude, rodrigues, true, &J1 );

        Eigen::Matrix3d RW2C = m_camera_state.attitude.inverse().toRotationMatrix();

        // TODO!

        const Eigen::Vector3d translation = -( RW2C * m_camera_state.position );

        //Eigen::Matrix3d J3 = 

        cv::Mat jacobian;
        cv::projectPoints(
            to_project,
            cv::Mat(3, 1, CV_64F, rodrigues.data()),
            cv::Mat(3, 1, CV_64F, translation.data()),
            m_calibration_matrix,
            m_distortion_coefficients,
            projected,
            jacobian);

        if( jacobian.rows != 2*num_visible ) throw std::runtime_error("internal error");
        if( jacobian.cols != 15 ) throw std::runtime_error("internal error");

        Eigen::Map< Eigen::MatrixXd > map(jacobian.ptr(), 2*num_visible, 15);

        J.reserve(20*num_visible);
        landmarks_descriptors.create(num_visible, m_descriptor->descriptorSize(), m_descriptor->descriptorType());

        for(int i=0; i<num_visible; i++)
        {
            // check that selection vector is sorted (if not, filling the sparse matrix would take a long time).

            if(i > 0 && selection[i-1] >= selection[i]) throw std::logic_error("internal error");

            const int j = selection[i];

            landmarks_descriptors.row(i) = m_landmarks[j].descriptor;

            Eigen::Matrix<double, 2, 3> jac_translation;
            jac_translation <<
                jacobian.at<float>(0,0), jacobian.at<float>(
            Eigen::Matrix<double, 2, 9> local_J;

            // TODO : fill J.
            J.insert( 2*i+0, 0 )
        }
    }

    // compute keypoints and their descriptors.

    if( num_visible == 0 )
    {
        num_found = 0;
        found.assign(num_landmarks, false);
    }
    else
    {
        cv::Mat& image = m_current_image.refFrame();

        cv::Mat mask(image.size(), CV_8U);
        mask = 0;

        Eigen::Matrix3d Q;
        Q <<
            measurement_sigma*measurement_sigma, 0.0,
            0.0, measurement_sigma*measurement_sigma;

        for(int i=0; i<num_visible; i++)
        {
            const int j = selection[i];

            const Eigen::Matrix2d covar_obs = J.block(2*j,0,2,dim) * m_state_covariance * J.block(2*j,0,2,dim).transpose() + Q;

            const double sigmax = std::sqrt( covar_obs(0,0) );
            const double sigmay = std::sqrt( covar_obs(1,1) );

            cv::Rect patch( projected[i].x - 3.0*sigmax, projected[i].y - 3.0*sigmay, 6.0*sigmax, 6.0*sigmay );

            if( (patch | viewport) == viewport )
            {
                mask(patch) = 1;
            }
        }

        m_detector->detect(image, corners, mask);
        m_descriptor->compute(image, corners, corner_descriptors);

        ;
    }

    // TODO: fill num_found, found and residuals.

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
            for(int i=0; i<num_landmarks; i++)
            {
                if(found[i])
                {
                    m_landmarks[i].last_seen_frame = m_frame_id;
                    proj.insert(2*j+0, 2*i+0) = 1.0;
                    proj.insert(2*j+1, 2*i+1) = 1.0;

                    noise.insert(2*j+0, 2*j+0) = measurement_sigma*measurement_sigma;
                    noise.insert(2*j+1, 2*j+1) = measurement_sigma*measurement_sigma;

                    j++;
                }
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


SLAMEngine* SLAMEngine::create()
{
    return new SLAMEngineImpl();
}


#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/features2d.hpp>
#include <iostream>
#include <QTime>
#include <QDebug>
#include "SLAMEngineImpl.h"
#include "FinitePriorityQueue.h"
#include "Tracker.h"

#define SLAM_DEBUG

SLAMEngineImpl::SLAMEngineImpl()
{
}

void SLAMEngineImpl::run()
{
    std::cout << "================" << std::endl;
    std::cout << " ENGINE STARTED " << std::endl;
    std::cout << "================" << std::endl;

    setup();

    m_camera->open();

    bool first = true;

    while( isInterruptionRequested() == false )
    {
        m_camera->read(m_current_image);

        if(m_current_image.isValid())
        {
            std::cout << "-> Processing frame " << m_frame_id << std::endl;

            // we skip first frame so that m_time_last_frame is filled.
            if( first )
            {
                first = false;
            }
            else
            {
                processImage();
                writeOutput();
            }

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

    std::cout << "ENGINE ENDED" << std::endl;
}

void SLAMEngineImpl::setup()
{
    // TODO
    m_measurement_standard_deviation = 1.8/640.0;

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

    m_mode = MODE_LOST;
    m_current_image.setValid(false);
    m_time_last_frame = 0.0;
    m_frame_id = 0;

    m_belief_mean.setZero();
    m_belief_mean(3) = 1.0;

    m_belief_covariance.setIdentity();

    m_camera_state.position.setZero();
    m_camera_state.attitude.setIdentity();
    m_camera_state.linear_velocity.setZero();
    m_camera_state.angular_velocity.setZero();

    m_tracker.setUnitLength( m_parameters.initialization_target_scale );
}

void SLAMEngineImpl::processImage()
{
    m_tracker.track( m_current_image.refFrame() );

    if( m_tracker.found() )
    {
        localizationPnP();
    }
    else
    {
        m_mode = MODE_TARGET_NOT_FOUND;
    }
}

void SLAMEngineImpl::localizationPnP()
{
    cv::Mat pnp_rodrigues;
    cv::Mat pnp_translation;

    bool ok = m_tracker.found();

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

    if(ok)
    {
        convertPose( pnp_rodrigues, pnp_translation, m_camera_state.attitude, m_camera_state.position );
        m_camera_state.linear_velocity.setZero();
        m_camera_state.angular_velocity.setZero();

        m_mode = MODE_TRACKING;
    }
    else
    {
        m_mode = MODE_LOST;
    }
}

void SLAMEngineImpl::localizationEKF()
{
    /*
    bool ok = ( m_mode == MODE_TRACKING && m_tracker.found() );

    // EKF prediction.

    if(ok)
    {
        const std::vector<cv::Point3f>& object_points = m_tracker.objectPoints();
        const std::vector<cv::Point2f>& image_points = m_tracker.imagePoints();
    }

    // EKF update.

    if(ok)
    {
    }

    if( ok == false )
    {
        m_mode = MODE_LOST;
    }
    */
}

/*
void SLAMEngineImpl::EKFPredict()
{
    const double dt = m_current_image.getTimestamp() - m_time_last_frame;

    const int num_landmarks = m_landmarks.size();

    const int dim = 13 + 3*num_landmarks;

    // TODO: setup these constants correctly.
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

    Eigen::VectorXd new_mu;
    Eigen::SparseMatrix<double> J;

    SLAMPrimitives::compute_f(mu, dt, new_mu, J);

    Eigen::MatrixXd new_sigma = J * (sigma + Q) * J.transpose();

    mu.swap(new_mu);
    sigma.swap(new_sigma);
}
*/

/*
void SLAMEngineImpl::EKFUpdate()
{
    const int num_landmarks = m_landmarks.size();

    const int dim = 13 + 3*num_landmarks;

    const double measurement_sigma = m_measurement_standard_deviation*m_current_image.width();

    const cv::Rect viewport( 0, 0, m_current_image.width(), m_current_image.height() );

    std::vector<int> selection;
    Eigen::VectorXd h;
    Eigen::SparseMatrix<double> J;

    SLAMPrimitives::compute_h(
        mu,
        m_calibration_matrix,
        m_distortion_coefficients,
        m_parameters.min_distance_to_camera,
        viewport,
        selection,
        h,
        J);

    Eigen::VectorXd residuals;

    computeResiduals(
        mu,
        sigma,
        selection,
        h,
        J,
        residuals);

    const int num_found = selection.size();

    std::cout << "Number of landmarks found in current frame: " << num_found << std::endl;

    if(num_found > 0)
    {
        Eigen::SparseMatrix<double> Q(2*num_found, 2*num_found);
        Q.reserve(2*num_found);

        for(int i=0; i<2*num_found; i++)
        {
            Q.insert(i,i) = measurement_sigma*measurement_sigma;
        }

        Eigen::MatrixXd S = J * sigma * J.transpose() + Q;

        Eigen::LDLT< Eigen::MatrixXd > solver;
        solver.compute( S );

        Eigen::VectorXd new_mu = mu + sigma * J.transpose() * solver.solve( residuals );
        Eigen::MatrixXd new_sigma = sigma - sigma * J.transpose() * solver.solve( J * sigma );

        mu.swap(new_mu);
        sigma.swap(new_sigma);
    }
}
*/

void SLAMEngineImpl::writeOutput()
{
    cv::Mat output_image = m_current_image.refFrame().clone();

    /*
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

    }
    */

    std::vector<SLAMOutputLandmark> output_landmarks;
    if( m_tracker.found() )
    {
        for(cv::Point3f pt : m_tracker.objectPoints())
        {
            SLAMOutputLandmark olm;
            olm.position << pt.x, pt.y, pt.z;
            output_landmarks.push_back(olm);
        }
    }
    /*
    output_landmarks.reserve( m_landmarks.size() );
    for(Landmark& lm : m_landmarks)
    {
        SLAMOutputLandmark olm;
        olm.position = lm.position;

        output_landmarks.push_back(olm);
    }
    */

    m_output->beginWrite();

    switch( m_mode )
    {
    case MODE_TARGET_NOT_FOUND:
        m_output->mode = "TARGET_NOT_FOUND";
        break;
    case MODE_LOST:
        m_output->mode = "LOST";
        break;
    case MODE_TRACKING:
        m_output->mode = "TRACKING";
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

/*
void SLAMPrimitives::compute_f(
    const Eigen::Matrix<double, 13, 1>& X,
    double dt,
    Eigen::Matrix<double, 13, 1>& f,
    Eigen::SparseMatrix<double>& J)
{
    const double x1 = X(0);
    const double x2 = X(1);
    const double x3 = X(2);

    const double a0 = X(3);
    const double a1 = X(4);
    const double a2 = X(5);
    const double a3 = X(6);

    const double v1 = X(7);
    const double v2 = X(8);
    const double v3 = X(9);

    double w1 = X(10);
    double w2 = X(11);
    double w3 = X(12);

    // TODO: handle small rotations in a better way.

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

    // fill f.

    f.resize(dim);

    f.head<13>() <<
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

    // fill J.

    Eigen::SparseMatrix<double, Eigen::RowMajor> Jrm;

    Jrm.resize(13, 13);
    Jrm.setZero();
    Jrm.reserve(40);

    // position.

    Jrm.insert(0,0) = 1.0;
    Jrm.insert(0,7) = dt;
    Jrm.insert(1,1) = 1.0;
    Jrm.insert(1,8) = dt;
    Jrm.insert(2,2) = 1.0;
    Jrm.insert(2,9) = dt;

    // attitude.

    // Generated automatically by python script system2.py.
    // BEGIN
    Jrm.insert(3,3) = r0;
    Jrm.insert(3,4) = -r1;
    Jrm.insert(3,5) = -r2;
    Jrm.insert(3,6) = -r3;
    Jrm.insert(3,10) = -0.5*a0*dt*r1 - 0.5*a1*dt*axis1*axis1*cos_theta + 1.0*a1*axis1*axis1*sin_theta_over_norm_w - a1*norm_w*sin_theta - 0.5*a2*dt*axis1*axis2*cos_theta + 1.0*a2*axis1*axis2*sin_theta_over_norm_w - 0.5*a3*dt*axis1*axis3*cos_theta + 1.0*a3*axis1*axis3*sin_theta_over_norm_w;
    Jrm.insert(3,11) = -0.5*a0*dt*r2 - 0.5*a1*dt*axis1*axis2*cos_theta + 1.0*a1*axis1*axis2*sin_theta_over_norm_w - 0.5*a2*dt*axis2*axis2*cos_theta + 1.0*a2*axis2*axis2*sin_theta_over_norm_w - a2*norm_w*sin_theta - 0.5*a3*dt*axis2*axis3*cos_theta + 1.0*a3*axis2*axis3*sin_theta_over_norm_w;
    Jrm.insert(3,12) = -0.5*a0*dt*r3 - 0.5*a1*dt*axis1*axis3*cos_theta + 1.0*a1*axis1*axis3*sin_theta_over_norm_w - 0.5*a2*dt*axis2*axis3*cos_theta + 1.0*a2*axis2*axis3*sin_theta_over_norm_w - 0.5*a3*dt*axis3*axis3*cos_theta + 1.0*a3*axis3*axis3*sin_theta_over_norm_w - a3*norm_w*sin_theta;
    Jrm.insert(4,3) = r1;
    Jrm.insert(4,4) = r0;
    Jrm.insert(4,5) = r3;
    Jrm.insert(4,6) = -r2;
    Jrm.insert(4,10) = 0.5*a0*dt*axis1*axis1*cos_theta - 1.0*a0*axis1*axis1*sin_theta_over_norm_w + a0*norm_w*sin_theta - 0.5*a1*dt*r1 + 0.5*a2*dt*axis1*axis3*cos_theta - 1.0*a2*axis1*axis3*sin_theta_over_norm_w - 0.5*a3*dt*axis1*axis2*cos_theta + 1.0*a3*axis1*axis2*sin_theta_over_norm_w;
    Jrm.insert(4,11) = 0.5*a0*dt*axis1*axis2*cos_theta - 1.0*a0*axis1*axis2*sin_theta_over_norm_w - 0.5*a1*dt*r2 + 0.5*a2*dt*axis2*axis3*cos_theta - 1.0*a2*axis2*axis3*sin_theta_over_norm_w - 0.5*a3*dt*axis2*axis2*cos_theta + 1.0*a3*axis2*axis2*sin_theta_over_norm_w - a3*norm_w*sin_theta;
    Jrm.insert(4,12) = 0.5*a0*dt*axis1*axis3*cos_theta - 1.0*a0*axis1*axis3*sin_theta_over_norm_w - 0.5*a1*dt*r3 + 0.5*a2*dt*axis3*axis3*cos_theta - 1.0*a2*axis3*axis3*sin_theta_over_norm_w + a2*norm_w*sin_theta - 0.5*a3*dt*axis2*axis3*cos_theta + 1.0*a3*axis2*axis3*sin_theta_over_norm_w;
    Jrm.insert(5,3) = r2;
    Jrm.insert(5,4) = -r3;
    Jrm.insert(5,5) = r0;
    Jrm.insert(5,6) = r1;
    Jrm.insert(5,10) = 0.5*a0*dt*axis1*axis2*cos_theta - 1.0*a0*axis1*axis2*sin_theta_over_norm_w - 0.5*a1*dt*axis1*axis3*cos_theta + 1.0*a1*axis1*axis3*sin_theta_over_norm_w - 0.5*a2*dt*r1 + 0.5*a3*dt*axis1*axis1*cos_theta - 1.0*a3*axis1*axis1*sin_theta_over_norm_w + a3*norm_w*sin_theta;
    Jrm.insert(5,11) = 0.5*a0*dt*axis2*axis2*cos_theta - 1.0*a0*axis2*axis2*sin_theta_over_norm_w + a0*norm_w*sin_theta - 0.5*a1*dt*axis2*axis3*cos_theta + 1.0*a1*axis2*axis3*sin_theta_over_norm_w - 0.5*a2*dt*r2 + 0.5*a3*dt*axis1*axis2*cos_theta - 1.0*a3*axis1*axis2*sin_theta_over_norm_w;
    Jrm.insert(5,12) = 0.5*a0*dt*axis2*axis3*cos_theta - 1.0*a0*axis2*axis3*sin_theta_over_norm_w - 0.5*a1*dt*axis3*axis3*cos_theta + 1.0*a1*axis3*axis3*sin_theta_over_norm_w - a1*norm_w*sin_theta - 0.5*a2*dt*r3 + 0.5*a3*dt*axis1*axis3*cos_theta - 1.0*a3*axis1*axis3*sin_theta_over_norm_w;
    Jrm.insert(6,3) = r3;
    Jrm.insert(6,4) = r2;
    Jrm.insert(6,5) = -r1;
    Jrm.insert(6,6) = r0;
    Jrm.insert(6,10) = 0.5*a0*dt*axis1*axis3*cos_theta - 1.0*a0*axis1*axis3*sin_theta_over_norm_w + 0.5*a1*dt*axis1*axis2*cos_theta - 1.0*a1*axis1*axis2*sin_theta_over_norm_w - 0.5*a2*dt*axis1*axis1*cos_theta + 1.0*a2*axis1*axis1*sin_theta_over_norm_w - a2*norm_w*sin_theta - 0.5*a3*dt*r1;
    Jrm.insert(6,11) = 0.5*a0*dt*axis2*axis3*cos_theta - 1.0*a0*axis2*axis3*sin_theta_over_norm_w + 0.5*a1*dt*axis2*axis2*cos_theta - 1.0*a1*axis2*axis2*sin_theta_over_norm_w + a1*norm_w*sin_theta - 0.5*a2*dt*axis1*axis2*cos_theta + 1.0*a2*axis1*axis2*sin_theta_over_norm_w - 0.5*a3*dt*r2;
    Jrm.insert(6,12) = 0.5*a0*dt*axis3*axis3*cos_theta - 1.0*a0*axis3*axis3*sin_theta_over_norm_w + a0*norm_w*sin_theta + 0.5*a1*dt*axis2*axis3*cos_theta - 1.0*a1*axis2*axis3*sin_theta_over_norm_w - 0.5*a2*dt*axis1*axis3*cos_theta + 1.0*a2*axis1*axis3*sin_theta_over_norm_w - 0.5*a3*dt*r3;
    // END

    // linear velocity.

    Jrm.insert(7, 7) = 1.0;
    Jrm.insert(8, 8) = 1.0;
    Jrm.insert(9, 9) = 1.0;

    // angular velocity.

    Jrm.insert(10, 10) = 1.0;
    Jrm.insert(11, 11) = 1.0;
    Jrm.insert(12, 12) = 1.0;

    Jrm.makeCompressed();

    J = Jrm;
}

void SLAMPrimitives::compute_h(
    const Eigen::Matrix<double,13,1>& X,
    const cv::Mat& camera_matrix,
    const cv::Mat& distortion_coefficients,
    const std::vector<cv::Point3f>& object_points,
    Eigen::VectorXd& h,
    Eigen::SparseMatrix<double>& J)
{
    // retrieve state dimension and number of landmarks.

    const int dim = X.size();

    if( ( dim - 13) % 3 != 0 ) throw std::runtime_error("internal error");

    const int num_landmarks = (dim - 13)/3;

    // retrieve camera position and attitude.

    const Eigen::Vector3d& camera_position = X.head<3>();

    Eigen::Quaterniond camera_attitude;
    camera_attitude.w() = X(3);
    camera_attitude.vec() = X.segment<3>(4);

    // prepare some data structures.

    visible_landmarks.clear();
    visible_landmarks.reserve(num_landmarks);

    std::vector<cv::Point3f> to_project;
    to_project.reserve(num_landmarks);

    // find which points we will try to project.

    for(int i=0; i<num_landmarks; i++)
    {
        const Eigen::Vector3d in_camera_frame = camera_attitude.inverse() * ( X.segment<3>(13 + 3*i) - camera_position );

        if( in_camera_frame.z() > 1.0e-7 && in_camera_frame.z() > min_distance_to_camera )
        {
            cv::Mat pixels = camera_matrix * ( cv::Mat_<float>(3,1) << in_camera_frame.x(), in_camera_frame.y(), in_camera_frame.z() );

            if( std::fabs(pixels.at<float>(2)) > 1.0e-7 )
            {
                cv::Point2f pt(
                    pixels.at<float>(0) / pixels.at<float>(2),
                    pixels.at<float>(1) / pixels.at<float>(2) );

                if( viewport.contains(pt) )
                {
                    visible_landmarks.push_back(i);

                    to_project.push_back( cv::Point3f(
                        in_camera_frame.x(),
                        in_camera_frame.y(),
                        in_camera_frame.z() ));
                }
            }
        }
    }

    const int num_visible = visible_landmarks.size();

    if( to_project.size() != num_visible ) throw std::runtime_error("internal error");

    h.resize(2*num_visible);

    J.setZero();
    J.resize(2*num_visible, dim);
    J.reserve(20*num_visible);

    if( num_visible > 0 )
    {
        std::vector<cv::Point2f> projected;
        cv::Mat jacobian;

        cv::projectPoints(
            to_project,
            cv::Mat::zeros(3, 1, CV_64F),
            cv::Mat::zeros(3, 1, CV_64F),
            camera_matrix,
            distortion_coefficients,
            projected,
            jacobian);

        if( jacobian.depth() != CV_64F ) throw std::runtime_error("internal error");
        if( jacobian.channels() != 1 ) throw std::runtime_error("internal error");
        if( jacobian.rows != 2*num_visible ) throw std::runtime_error("internal error");
        if( projected.size() != num_visible) throw std::runtime_error("internal error");

        Eigen::Map< Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> > J1( (double*) jacobian.ptr(), 2*num_visible, 15);

        for(int i=0; i<num_visible; i++)
        {
            // check that visible_landmarks vector is sorted (if not, filling the sparse matrix would take a long time).
            if( i > 0 && visible_landmarks[i-1] >= visible_landmarks[i] ) throw std::logic_error("internal error");

            const int j = visible_landmarks[i];

            // set jacobian coefficients.

            Eigen::Matrix<double, 3, 10> J2;
            computeJacobianOfWorld2CameraTransformation(X, j, J2 );

            Eigen::Matrix<double, 2, 10> J3 = J1.block<2, 3>(2*i, 3) * J2;

            for(int a=0; a<2; a++)
            {
                for(int b=0; b<3; b++)
                    J.insert(2*i+a, b) = J3(a, b);

                for(int b=0; b<4; b++)
                    J.insert(2*i+a, 3+b) = J3(a, 3+b);

                for(int b=0; b<3; b++)
                    J.insert(2*i+a, 13+3*j+b) = J3(a, 7+b);
            }
        }
    }

    J.makeCompressed();
}

void SLAMPrimitives::computeJacobianOfWorld2CameraTransformation(
    const Eigen::VectorXd& X,
    int landmark,
    Eigen::Matrix<double, 3, 10>& J)
{
    Eigen::Quaterniond camera_attitude;
    camera_attitude.w() = X(3);
    camera_attitude.vec() = X.segment<3>(4);

    const Eigen::Matrix3d R = camera_attitude.inverse().toRotationMatrix();

    const double x1 = X(0);
    const double x2 = X(1);
    const double x3 = X(2);

    const double a0 = X(3);
    const double a1 = X(4);
    const double a2 = X(5);
    const double a3 = X(6);

    const double y1 = X(13+3*landmark+0);
    const double y2 = X(13+3*landmark+1);
    const double y3 = X(13+3*landmark+2);

    J.block<3,3>(0, 0) = -R;

    J(0, 3) = -2*a2*(-x3 + y3) + 2*a3*(-x2 + y2);
    J(0, 4) = 2*a2*(-x2 + y2) + 2*a3*(-x3 + y3);
    J(0, 5) = -2*a0*(-x3 + y3) + 2*a1*(-x2 + y2) - 4*a2*(-x1 + y1);
    J(0, 6) = 2*a0*(-x2 + y2) + 2*a1*(-x3 + y3) - 4*a3*(-x1 + y1);

    J(1, 3) = 2*a1*(-x3 + y3) - 2*a3*(-x1 + y1);
    J(1, 4) = 2*a0*(-x3 + y3) - 4*a1*(-x2 + y2) + 2*a2*(-x1 + y1);
    J(1, 5) = 2*a1*(-x1 + y1) + 2*a3*(-x3 + y3);
    J(1, 6) = -2*a0*(-x1 + y1) + 2*a2*(-x3 + y3) - 4*a3*(-x2 + y2);

    J(2, 3) = -2*a1*(-x2 + y2) + 2*a2*(-x1 + y1);
    J(2, 4) = -2*a0*(-x2 + y2) - 4*a1*(-x3 + y3) + 2*a3*(-x1 + y1);
    J(2, 5) = 2*a0*(-x1 + y1) - 4*a2*(-x3 + y3) + 2*a3*(-x2 + y2);
    J(2, 6) = 2*a1*(-x1 + y1) + 2*a2*(-x2 + y2);

    J.block<3,3>(0, 7) = R;
}
*/

void SLAMEngineImpl::convertPose(
    const cv::Mat& rodrigues,
    const cv::Mat& t,
    Eigen::Quaterniond& attitude,
    Eigen::Vector3d& position)
{
    Eigen::Vector3d rodrigues_eigen;
    if( rodrigues.type() == CV_32F )
    {
        rodrigues_eigen.x() = rodrigues.at<float>(0);
        rodrigues_eigen.y() = rodrigues.at<float>(1);
        rodrigues_eigen.z() = rodrigues.at<float>(2);
    }
    else if( rodrigues.type() == CV_64F )
    {
        rodrigues_eigen.x() = rodrigues.at<double>(0);
        rodrigues_eigen.y() = rodrigues.at<double>(1);
        rodrigues_eigen.z() = rodrigues.at<double>(2);
    }

    Eigen::Vector3d t_eigen;
    if( t.type() == CV_32F )
    {
        t_eigen.x() = t.at<float>(0);
        t_eigen.y() = t.at<float>(1);
        t_eigen.z() = t.at<float>(2);
    }
    else if( t.type() == CV_64F )
    {
        t_eigen.x() = t.at<double>(0);
        t_eigen.y() = t.at<double>(1);
        t_eigen.z() = t.at<double>(2);
    }

    const double norm = rodrigues_eigen.norm();

    if( norm > 1.0e-8 )
    {
        attitude.vec() = -sin(0.5*norm) * rodrigues_eigen / norm;
        attitude.w() = cos(0.5*norm);
    }
    else
    {
        attitude.setIdentity();
    }

    position = -( attitude * t_eigen );
}

SLAMEngine* SLAMEngine::create()
{
    return new SLAMEngineImpl();
}


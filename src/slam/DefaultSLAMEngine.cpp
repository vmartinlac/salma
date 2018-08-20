#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <iostream>
#include <QDebug>
#include "DefaultSLAMEngine.h"
#include "target.h"

DefaultSLAMEngine::DefaultSLAMEngine()
{
}

void DefaultSLAMEngine::run()
{
    // start-up
    {
        if( !m_camera ) throw std::runtime_error("No camera was set.");

        cv::Mat_<float> K(3,3);
        K <<
            m_parameters.fx, 0.0, m_parameters.cx,
            0.0, m_parameters.fy, m_parameters.cy,
            0.0, 0.0, 1.0;

        cv::Mat_<float> lens_distortion(5, 1);
        lens_distortion <<
            m_parameters.distortion_k1,
            m_parameters.distortion_k2,
            m_parameters.distortion_p1,
            m_parameters.distortion_p2,
            m_parameters.distortion_k3;

        //m_mode = MODE_DEAD;
        m_mode = MODE_INIT;
        m_calibration_matrix = K;
        m_distortion_coefficients = lens_distortion;
        m_camera_state.position.setZero();
        m_camera_state.attitude.setIdentity();
        m_camera_state.linear_velocity.setZero();
        m_camera_state.angular_velocity.setZero();
        m_landmarks.clear();
        m_state_covariance.resize(0, 0);
        m_candidate_landmarks.clear();
        m_time_last_frame = 0.0;
    }

    m_camera->open();

    bool first = true;

    while( isInterruptionRequested() == false )
    {
        m_camera->read(m_current_image);

        if(m_current_image.isValid())
        {
            if(first)
            {
                first = false;
            }
            else
            {
                switch(m_mode)
                {
                case MODE_INIT:
                    processImageInit();
                    break;
                case MODE_SLAM:
                    processImageSLAM();
                    break;
                case MODE_DEAD:
                default:
                    processImageDead();
                    break;
                }
            }

            write_output();

            m_time_last_frame = m_current_image.getTimestamp();
        }
        else
        {
            QThread::msleep(5);
            //QThread::yieldCurrentThread();
        }
    }

    m_camera->close();
}

void DefaultSLAMEngine::processImageInit()
{
    bool ok;
    target::Detector d;
    std::vector< cv::Point3f > object_points;
    std::vector< cv::Point2f > image_points;
    cv::Mat pnp_rodrigues;
    cv::Mat pnp_translation;
    target::KindOfTarget kind_of_target;
    
    switch(m_parameters.initialization_target_kind)
    {
    case SLAMParameters::INITIALIZATION_TARGET_TWO_PLANE:
        kind_of_target = target::TWO_PLANES;
        break;
    case SLAMParameters::INITIALIZATION_TARGET_ONE_PLANE:
    default:
        kind_of_target = target::ONE_PLANE;
        break;
    }

    ok = d.run(
        m_current_image.refFrame(),
        kind_of_target,
        m_parameters.initialization_target_scale,
        object_points,
        image_points);

    if( ok )
    {
        ok = ( object_points.size() >= 20 );
    }

    if( ok )
    {
        ok = cv::solvePnP(
            object_points,
            image_points,
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
        // create first landmarks.

        // Nice to have: check beforehand that we have enough landmarks far enough from the borders to avoid a little bit of computation waste.

        m_landmarks.clear();

        for(int i=0; i<object_points.size(); i++)
        {
            Landmark lm;

            const bool ret = extractPatch( image_points[i], lm.patch );

            if(ret)
            {
                lm.position.x() = object_points[i].x;
                lm.position.y() = object_points[i].y;
                lm.position.z() = object_points[i].z;
                lm.num_failed_detections = 0;
                lm.num_successful_detections = 1;
                lm.last_successful_detection_time = m_current_image.getTimestamp();

                m_landmarks.emplace_back(std::move(lm));
            }
        }

        // set mode to SLAM.

        if( m_landmarks.size() >= 8 )
        {
            const double sigma = 0.15*m_parameters.initialization_target_scale; // TODO: to clarify.

            m_state_covariance.resize(
                13 + 3*m_landmarks.size(),
                13 + 3*m_landmarks.size() );

            m_state_covariance.setZero();
            m_state_covariance.diagonal().fill(sigma*sigma); // TODO

            //m_mode = MODE_DEAD;
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

void DefaultSLAMEngine::processImageSLAM()
{
    cv::Mat greyscale;
    cv::cvtColor(m_current_image.refFrame(), greyscale, CV_BGR2GRAY);

    cv::goodFeaturesToTrack(
        greyscale,
        m_current_corners, 300, 0.05, 3);

    Eigen::VectorXd state_mu;
    Eigen::MatrixXd state_sigma;

    EKFPredict(state_mu, state_sigma);
    EKFUpdate(state_mu, state_sigma);
    saveState(state_mu, state_sigma);

    // TODO: manage candidate landmarks.
}

void DefaultSLAMEngine::write_output()
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

            const int radius = output_image.cols*8/640;
            cv::circle(output_image, pt, radius, cv::Scalar(0, 255, 0), -1);
        }
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
    m_output->image = output_image;
    m_output->position = m_camera_state.position;
    m_output->attitude = m_camera_state.attitude;
    m_output->linear_velocity = m_camera_state.linear_velocity;
    m_output->angular_velocity = m_camera_state.angular_velocity;
    m_output->landmarks.swap(output_landmarks);
    m_output->endWrite();
    m_output->updated();
}

void DefaultSLAMEngine::EKFPredict(Eigen::VectorXd& pred_mu, Eigen::MatrixXd& pred_sigma)
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

    Eigen::SparseMatrix<double> J(dim, dim);
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
    const double sigma_v = 0.0; //dt*0.1;
    const double sigma_w = 0.0;

    Eigen::SparseMatrix<double> Q(dim, dim);
    Q.reserve(6);
    Q.insert(7,7) = sigma_v;
    Q.insert(8,8) = sigma_v;
    Q.insert(9,9) = sigma_v;
    Q.insert(10,10) = sigma_w;
    Q.insert(11,11) = sigma_w;
    Q.insert(12,12) = sigma_w;

    Q.makeCompressed();

    pred_sigma = J * (m_state_covariance + Q) * J.transpose();
}

void DefaultSLAMEngine::EKFUpdate(Eigen::VectorXd& pred_mu, Eigen::MatrixXd& pred_sigma)
{
    const Image& image = m_current_image;

    const int num_landmarks = m_landmarks.size();
    const int dim = 13 + 3*num_landmarks;

    std::vector<bool> found(num_landmarks);

    Eigen::VectorXd residuals(2*num_landmarks);

    Eigen::SparseMatrix<double> J(2*num_landmarks, dim);
    J.reserve(20*num_landmarks);

    int num_found = 0;

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

        cv::Point2i found_point(0,0);

        bool ok = true;

        if( ok)
        {
            ok = ( ycam3 > m_parameters.min_distance_to_camera ); // TODO: check this line.
        }

        if( ok )
        {
            u1 = c1 + f1 * ycam1/ycam3;
            u2 = c2 + f2 * ycam2/ycam3;
            cv::Rect viewport( cv::Point2i(0,0), m_current_image.refFrame().size() );
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

            const double box_radius_1 = std::sqrt( covariance(0,0) );
            const double box_radius_2 = std::sqrt( covariance(1,1) );

            ok = findPatch(
                m_landmarks[i].patch,
                cv::Rect( u1-box_radius_1, u2-box_radius_2, 2*box_radius_1, 2*box_radius_2 ),
                found_point );
        }

        if(ok)
        {
            residuals(2*i+0) = found_point.x - u1;
            residuals(2*i+1) = found_point.y - u2;
            num_found++;
        }
        else
        {
            residuals(2*i+0) = 0.0;
            residuals(2*i+1) = 0.0;
        }

        found[i] = ok;
    }

    // TODO: remove this line.
    std::cout << num_found << std::endl;
    //

    if(num_found > 0)
    {
        // compute a matrix to extract out residuals corresponding to found landmarks.

        Eigen::SparseMatrix<double> proj(2*num_found, 2*num_landmarks);
        proj.reserve(4*num_found);

        int j = 0;
        for(int i=0; j<num_found && i<num_landmarks; i++)
        {
            if(found[i])
            {
                proj.insert(2*j+0, 2*i+0) = 1.0;
                proj.insert(2*j+1, 2*i+1) = 1.0;
                j++;
            }
        }

        Eigen::SparseMatrix<double> noise(2*num_found, 2*num_found);
        noise.reserve(2*num_found);
        for(int i=0; i<2*num_found; i++)
        {
            noise.insert(i, i) = 8.0*8.0; // TODO: define this constant somewhere else.
        }

        auto projected_J = proj * J;
        auto projected_residuals = proj * residuals;

        Eigen::MatrixXd S = projected_J * pred_sigma * projected_J.transpose() + noise;

        Eigen::FullPivLU< Eigen::MatrixXd > solver;
        solver.compute( S );

        Eigen::VectorXd new_mu = pred_mu + pred_sigma * projected_J.transpose() * solver.solve( projected_residuals );
        Eigen::MatrixXd new_sigma = pred_sigma - pred_sigma * projected_J.transpose() * solver.solve( projected_J * pred_sigma );

        pred_mu.swap(new_mu);
        pred_sigma.swap(new_sigma);
    }
}

void DefaultSLAMEngine::saveState(Eigen::VectorXd& mu, Eigen::MatrixXd& sigma)
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

void DefaultSLAMEngine::processImageDead()
{
    m_output->beginWrite();
    m_output->image = m_current_image.refFrame().clone();
    m_output->mode = "DEAD";
    m_output->endWrite();
    m_output->updated();
}

bool DefaultSLAMEngine::extractPatch( const cv::Point2i& point, cv::Mat& patch)
{
    const cv::Mat& frame = m_current_image.refFrame();

    const int radius = m_parameters.patch_size/2;

    cv::Rect image_rect( cv::Point2i(0, 0), frame.size() );
    cv::Rect patch_rect( cv::Point2i( point.x-radius, point.y-radius), cv::Size( m_parameters.patch_size, m_parameters.patch_size ) );

    bool ret = false;

    if( (image_rect & patch_rect) == patch_rect )
    {
        patch = frame(patch_rect);
        ret = true;
    }

    return ret;
}

bool DefaultSLAMEngine::findPatch(
    const cv::Mat& patch,
    const cv::Rect& area,
    cv::Point2i& result)
{
    const cv::Mat& image = m_current_image.refFrame();

    bool found = false;

    cv::Mat candidate;

    for(std::vector<cv::Point2i>::iterator it=m_current_corners.begin(); found == false && it!=m_current_corners.end(); it++)
    {
        if( area.contains(*it) && extractPatch( *it, candidate) && comparePatches(candidate, patch) )
        {
            result = *it;
            found = true;
        }
    }

    return found;
}

bool DefaultSLAMEngine::comparePatches(const cv::Mat& P1, const cv::Mat& P2)
{
    if(
        P1.rows != m_parameters.patch_size || P1.cols != m_parameters.patch_size ||
        P2.rows != m_parameters.patch_size || P2.cols != m_parameters.patch_size )
    {
        throw std::runtime_error("internal error");
    }

    const int N = m_parameters.patch_size * m_parameters.patch_size;

    const double dist = cv::norm(P1, P2, cv::NORM_L2) / double(N);

    //std::cout << dist << std::endl;
    return dist < 12.0; // TODO: define this constant somewhere else.
}

SLAMEngine* SLAMEngine::create()
{
    return new DefaultSLAMEngine();
}

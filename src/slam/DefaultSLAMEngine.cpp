#include <opencv2/calib3d.hpp>
#include <iostream>
#include <QDebug>
#include "DefaultSLAMEngine.h"
#include "target.h"

DefaultSLAMEngine::DefaultSLAMEngine(Camera* camera, QObject* parent) :
    m_camera(camera),
    SLAMEngine(parent)
{
}

void DefaultSLAMEngine::run()
{
    m_mode = MODE_INIT;

    if( m_camera == nullptr ) throw std::runtime_error("Internal error");

    m_camera_state.position.setZero();
    m_camera_state.attitude.setIdentity();
    m_camera_state.linear_velocity.setZero();
    m_camera_state.angular_velocity.setZero();
    m_landmarks.clear();
    m_state_covariance.resize(0, 0);
    m_candidate_landmarks.clear();
    m_time_last_frame = 0.0;
    bool first = true;

    m_camera->open();

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

            m_time_last_frame = m_current_image.getTimestamp();
        }
        else
        {
            QThread::yieldCurrentThread();
        }
    }

    m_camera->close();
}

void DefaultSLAMEngine::processImageInit()
{
    bool ok;
    target::Detector d;
    cv::Mat samples;
    cv::Mat rodrigues_rotation;
    cv::Mat translation;

    ok = d.run(
        m_current_image.refFrame(),
        target::Detector::ONE_PLANE,
        m_parameters.calibration_target_scale,
        samples);

    if( ok )
    {
        ok = ( samples.rows >= 4*4 && samples.cols == 5 );
    }

    if( ok )
    {
        ok = cv::solvePnP(
            samples( cv::Range::all(), cv::Range(2, 5) ),
            samples( cv::Range::all(), cv::Range(0, 2) ),
            m_parameters.calibration_matrix,
            m_parameters.distortion_coefficients,
            rodrigues_rotation,
            translation,
            false,
            cv::SOLVEPNP_ITERATIVE );
    }

    if( ok )
    {
        // copy camera pose data.

        m_camera_state.position <<
            translation.at<float>(0, 0),
            translation.at<float>(1, 0),
            translation.at<float>(2, 0);

        Eigen::Vector3d rodrigues;
        rodrigues <<
            rodrigues_rotation.at<float>(0, 0),
            rodrigues_rotation.at<float>(1, 0),
            rodrigues_rotation.at<float>(2, 0);

        const double norm = rodrigues.norm();
        const double cos_half_theta = cos(0.5*norm);
        const double sin_half_theta = sin(0.5*norm);

        m_camera_state.attitude.coeffs() << 
            rodrigues(0)*sin_half_theta/norm,
            rodrigues(1)*sin_half_theta/norm,
            rodrigues(2)*sin_half_theta/norm,
            cos_half_theta ;

        // create first landmarks.

        // TODO : beforehand, check that we have enough landmarks far enough from the borders of the image so that they have a patch.

        m_landmarks.clear();

        for(int i=0; i<samples.rows; i++)
        {
            Landmark lm;

            lm.position <<
                samples.at<float>(i, 2),
                samples.at<float>(i, 3),
                samples.at<float>(i, 4);

            const bool ret = extractPatch(
                samples.at<float>(i, 0),
                samples.at<float>(i, 1),
                lm.patch);

            lm.num_failed_detections = 0;
            lm.num_successful_detections = 1;
            lm.last_successful_detection_time = m_current_image.getTimestamp();

            if(ret)
            {
                m_landmarks.emplace_back(std::move(lm));
            }
        }

        // set mode to SLAM.

        if( m_landmarks.size() >= 8 )
        {
            const double sigma = 0.15*m_parameters.calibration_target_scale; // TODO: to clarify.

            m_state_covariance.resize(
                12 + m_landmarks.size(),
                12 + m_landmarks.size() );

            m_state_covariance.setZero();
            m_state_covariance.diagonal().fill(sigma*sigma); // TODO

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
    cv::goodFeaturesToTrack(
        m_current_image.refFrame(),
        m_current_corners, 30, 0.01, 2);

    Eigen::VectorXd state_mu;
    Eigen::MatrixXd state_sigma;

    EKFPredict(state_mu, state_sigma);
    EKFUpdate(state_mu, state_sigma);
    saveState(state_mu, state_sigma);
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
    const double R21 = R12;
    const double R22 = 1.0 - 2.0*(a1*a1 + a3*a3);
    const double R23 = 2.0*(a2*a3 - a1*a0);
    const double R31 = R13;
    const double R32 = R23;
    const double R33 = 1.0 - 2.0*(a1*a1 + a2*a2);

    const double c1 = m_parameters.calibration_matrix.at<float>(0,2);
    const double c2 = m_parameters.calibration_matrix.at<float>(1,2);

    const double f1 = m_parameters.calibration_matrix.at<float>(0,0);
    const double f2 = m_parameters.calibration_matrix.at<float>(1,1);

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

        double found_u1 = 0.0;
        double found_u2 = 0.0;

        bool ok = true;

        if( ok)
        {
            ok = ( ycam3 > m_parameters.min_distance_to_camera );
        }

        if( ok )
        {
            u1 = c1 + f1 * ycam1/ycam3;
            u2 = c2 + f2 * ycam2/ycam3;
            // TODO : check that u1 and u2 lies on the image in order to spare some wasteful computations.
        }

        if( ok )
        {
            // Generated by python script measurement.py
            // BEGIN
            J.insert(2*i+0, 0) = f1*R13*ycam1/(ycam3*ycam3) + f1*(-R11)/ycam3;
            J.insert(2*i+0, 1) = f1*R23*ycam1/(ycam3*ycam3) + f1*(-R12)/ycam3;
            J.insert(2*i+0, 2) = f1*(-R13)/ycam3 + f1*R33*ycam1/(ycam3*ycam3);
            J.insert(2*i+0, 3) = f1*(2*a1*d2 - 2*a2*d1)*ycam1/(ycam3*ycam3) + f1*(2*a2*d3 - 2*a3*d2)/ycam3;
            J.insert(2*i+0, 4) = f1*(2*a2*d2 + 2*a3*d3)/ycam3 + f1*(2*a0*d2 + 4*a1*d3 - 2*a3*d1)*ycam1/(ycam3*ycam3);
            J.insert(2*i+0, 5) = f1*(-2*a0*d1 + 4*a2*d3 - 2*a3*d2)*ycam1/(ycam3*ycam3) + f1*(2*a0*d3 + 2*a1*d2 - 4*a2*d1)/ycam3;
            J.insert(2*i+0, 6) = f1*(-2*a1*d1 - 2*a2*d2)*ycam1/(ycam3*ycam3) + f1*(-2*a0*d2 + 2*a1*d3 - 4*a3*d1)/ycam3;
            J.insert(2*i+0, 13+3*i+0) = f1*(-R13)*ycam1/(ycam3*ycam3) + f1*R11/ycam3;
            J.insert(2*i+0, 13+3*i+1) = f1*(-R23)*ycam1/(ycam3*ycam3) + f1*R12/ycam3;
            J.insert(2*i+0, 13+3*i+2) = f1*R13/ycam3 + f1*(-R33)*ycam1/(ycam3*ycam3);
            J.insert(2*i+1, 0) = f2*R13*ycam2/(ycam3*ycam3) + f2*(-R12)/ycam3;
            J.insert(2*i+1, 1) = f2*R23*ycam2/(ycam3*ycam3) + f2*(2*a1*a1 + 2*a3*a3 - 1)/ycam3;
            J.insert(2*i+1, 2) = f2*(-R23)/ycam3 + f2*R33*ycam2/(ycam3*ycam3);
            J.insert(2*i+1, 3) = f2*(2*a1*d2 - 2*a2*d1)*ycam2/(ycam3*ycam3) + f2*(-2*a1*d3 - 2*a3*d1)/ycam3;
            J.insert(2*i+1, 4) = f2*(2*a0*d2 + 4*a1*d3 - 2*a3*d1)*ycam2/(ycam3*ycam3) + f2*(-2*a0*d3 - 4*a1*d2 + 2*a2*d1)/ycam3;
            J.insert(2*i+1, 5) = f2*(2*a1*d1 + 2*a3*d3)/ycam3 + f2*(-2*a0*d1 + 4*a2*d3 - 2*a3*d2)*ycam2/(ycam3*ycam3);
            J.insert(2*i+1, 6) = f2*(-2*a1*d1 - 2*a2*d2)*ycam2/(ycam3*ycam3) + f2*(-2*a0*d1 + 2*a2*d3 - 4*a3*d2)/ycam3;
            J.insert(2*i+1, 13+3*i+0) = f2*(-R13)*ycam2/(ycam3*ycam3) + f2*R12/ycam3;
            J.insert(2*i+1, 13+3*i+1) = f2*(-R23)*ycam2/(ycam3*ycam3) + f2*R22/ycam3;
            J.insert(2*i+1, 13+3*i+2) = f2*R23/ycam3 + f2*(-R33)*ycam2/(ycam3*ycam3);
            // END

            Eigen::Matrix2d covariance = J.block(2*i,0,2,dim) * pred_sigma * J.block(2*i,0,2,dim).transpose();

            const double box_radius_1 = std::sqrt( covariance(0,0) );
            const double box_radius_2 = std::sqrt( covariance(1,1) );

            ok = findPatch(
                m_landmarks[i].patch,
                u1, u2,
                box_radius_1, box_radius_2,
                found_u1, found_u2);
        }

        if(ok)
        {
            residuals(2*i+0) = found_u1 - u1;
            residuals(2*i+1) = found_u2 - u2;
            num_found++;
        }
        else
        {
            residuals(2*i+0) = 0.0;
            residuals(2*i+1) = 0.0;
        }

        found[i] = ok;
    }

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
        noise.diagonal().fill(6.0); // TODO: define this constant somewhere else.

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
}

bool DefaultSLAMEngine::extractPatch(float u, float v, cv::Mat& patch)
{
    const cv::Mat& image = m_current_image.refFrame();

    const int radius = m_parameters.patch_size/2;

    const int U = (int) cvRound(u);
    const int V = (int) cvRound(v);

    bool ret = false;

    cv::Rect viewport(
        cv::Point2f( radius+1, radius+1 ),
        cv::Size( image.cols-m_parameters.patch_size-1, image.rows-m_parameters.patch_size-1 ) );

    if( viewport.contains( cv::Point2i(U,V) ) )
    {
        patch = image(
            cv::Range(V-radius, V+radius+1),
            cv::Range(U-radius, U+radius+1) );

        ret = true;
    }

    return ret;
}

bool DefaultSLAMEngine::findPatch(
    const cv::Mat& patch,
    double box_center_u,
    double box_center_v,
    double box_radius_u,
    double box_radius_v,
    double& found_u,
    double& found_v)
{
    const cv::Mat& image = m_current_image.refFrame();

    cv::Rect area( cv::Point2i( box_center_u - box_radius_u ), cv::Size( 2*box_radius_u, 2*box_radius_v) );

    bool found = false;

    cv::Mat candidate;

    for(std::vector<cv::Point2i>::iterator it=m_current_corners.begin(); found == false && it!=m_current_corners.end(); it++)
    {
        if( area.contains(*it) && extractPatch(it->x, it->y, candidate) )
        {
            if( comparePatches(candidate, patch) )
            {
                found_u = it->x;
                found_v = it->y;
                found = true;
            }
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

    return dist < 12.0; // TODO: define this constant somewhere else.
}

SLAMEngine* SLAMEngine::create(Camera* camera)
{
    return new DefaultSLAMEngine(camera);
}


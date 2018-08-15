#include <opencv2/calib3d.hpp>
#include <iostream>
#include <QDebug>
#include "DefaultSLAMEngine.h"
#include "target.h"

DefaultSLAMEngine::DefaultSLAMEngine(Camera* camera) :
    m_camera(camera)
{
}

void DefaultSLAMEngine::retrieveParameters()
{
    throw; // TODO
    //m_parameters.calibration_matrix.setZero(); // TODO
    //m_parameters.distortion_coefficients.setZero(); // TODO
    m_parameters.patch_size = 11;
    m_parameters.target_unit_length = 1.0;
    m_parameters.num_depth_hypotheses = 100;
}

void DefaultSLAMEngine::run()
{
    /*
    {
        const double sx = 0.0;
        const double sv = 0.0;
        const double sw = 0.0; //M_PI*0.02/1.0;
        const double sx2 = sx*sx;
        const double sv2 = sv*sv;
        const double sw2 = sw*sw;
        m_camera_state.position.setZero();
        m_camera_state.attitude.setIdentity();
        m_camera_state.linear_velocity.setZero();
        //m_camera_state.linear_velocity << 1.0, 0.0, 0.0;
        m_camera_state.angular_velocity << 0.0, 0.0, M_PI*0.5;
        //m_camera_state.angular_velocity.setZero();
        m_landmarks.clear();
        m_state_covariance.resize(13, 13);
        m_state_covariance.setZero();
        m_state_covariance.diagonal() << sx2, sx2, sx2, 0.0, 0.0, 0.0, 0.0, sv2, sv2, sv2, sw2, sw2, sw2;
        m_candidate_landmarks.clear();
        m_time_last_frame = 0.0;
        Image img;
        img.setTimestamp(1.0);
        std::cout << "===============" << std::endl;
        processImageSLAM(img);
        std::cout << "===============" << std::endl;
        processImageSLAM(img);
        std::cout << "===============" << std::endl;
        processImageSLAM(img);
        abort();
    }
    */

    m_mode = MODE_INIT;

    retrieveParameters();

    if( m_camera == nullptr ) throw std::runtime_error("Internal error");

    m_camera_state.position.setZero();
    m_camera_state.attitude.setIdentity();
    m_camera_state.linear_velocity.setZero();
    m_camera_state.angular_velocity.setZero();
    m_landmarks.clear();
    m_state_covariance.resize(0, 0);
    m_candidate_landmarks.clear();
    m_time_last_frame = 0.0;

    m_camera->open();

    while( isInterruptionRequested() == false )
    {
        Image image;
        m_camera->read(image);

        if(image.isValid())
        {
            switch(m_mode)
            {
            case MODE_INIT:
                processImageInit(image);
                break;
            case MODE_SLAM:
                processImageSLAM(image);
                break;
            case MODE_DEAD:
            default:
                processImageDead(image);
                break;
            }

            m_time_last_frame = image.getTimestamp();
        }
        else
        {
            QThread::yieldCurrentThread();
        }
    }

    m_camera->close();
}

void DefaultSLAMEngine::processImageInit(Image& image)
{
    bool ok;
    target::Detector d;
    cv::Mat samples;
    cv::Mat rodrigues_rotation;
    cv::Mat translation;

    ok = d.run(
        image.refFrame(),
        target::Detector::ONE_PLANE,
        m_parameters.target_unit_length,
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
                image.refFrame(),
                samples.at<float>(i, 0),
                samples.at<float>(i, 1),
                lm.patch);

            if(ret)
            {
                m_landmarks.emplace_back(std::move(lm));
            }
        }

        // set mode to SLAM.

        if( m_landmarks.size() >= 8 )
        {
            const double sigma = 0.15*m_parameters.target_unit_length; // TODO: to clarify.

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

void DefaultSLAMEngine::processImageSLAM(Image& image)
{
    // BEGIN TMP
    /*
    {
        std::cout << "state:" << std::endl;
        std::cout << m_camera_state.position.transpose() << std::endl;
        std::cout << m_camera_state.attitude.coeffs().transpose() << std::endl;
        std::cout << m_camera_state.linear_velocity.transpose() << std::endl;
        std::cout << m_camera_state.angular_velocity.transpose() << std::endl;
        std::cout << m_state_covariance << std::endl;
        std::cout << std::endl;
    }
    */
    // END TMP

    const double dt = image.getTimestamp() - m_time_last_frame;
    const int num_landmarks = m_landmarks.size();
    const int dim = 13 + 3*num_landmarks;

    // prediction.

    Eigen::VectorXd pred_mu;
    Eigen::MatrixXd pred_sigma;

    {
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

        // J est la jacobienne de la fonction de transition vers le nouvel état.

        Eigen::SparseMatrix<double> J(dim, dim);
        J.reserve(2*3+7*4+1*6+1*num_landmarks*3);

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

        //Eigen::MatrixXd tmp = J;
        //std::cout << "J = " << std::endl << tmp << std::endl;

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

    // Compute observations.
}

void DefaultSLAMEngine::processImageDead(Image& image)
{
}

bool DefaultSLAMEngine::extractPatch(cv::Mat& image, float x, float y, cv::Mat& patch)
{
    const int radius = m_parameters.patch_size/2;

    const int X = (int) cvRound(x);
    const int Y = (int) cvRound(y);

    bool ret = false;

    if( 0 <= X-radius && X+radius < image.cols && 0 <= Y-radius && Y+radius < image.rows)
    {
        patch = image(
            cv::Range(Y-radius, Y+radius+1),
            cv::Range(X-radius, X+radius+1) );

        ret = true;
    }

    return ret;
}

SLAMEngine* SLAMEngine::create(Camera* camera)
{
    return new DefaultSLAMEngine(camera);
}


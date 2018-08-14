#include <opencv2/calib3d.hpp>
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
    m_mode = MODE_INIT;

    retrieveParameters();

    if( m_camera == nullptr ) throw std::runtime_error("Internal error");

    m_camera_state.position.setZero();
    m_camera_state.attitude.setIdentity();
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
    const double dt = image.getTimestamp() - m_time_last_frame;
    const int num_landmarks = m_landmarks.size();
    const int dim = 13 + 3*num_landmarks;

    // prediction.

    Eigen::VectorXd pred_mu;
    Eigen::MatrixXd pred_sigma;

    {
        Eigen::Quaterniond omega;
        omega.w() = 0.0;
        omega.vec() = m_camera_state.angular_velocity;

        Eigen::Quaterniond pred_attitude;
        pred_attitude = m_camera_state.attitude.coeffs() + 0.5*dt* ( omega*m_camera_state.attitude ).coeffs();
        pred_attitude.normalize();

        pred_mu.resize(dim);

        pred_mu.segment<3>(0) = m_camera_state.position + dt*m_camera_state.linear_velocity;
        pred_mu.segment<4>(3) = pred_attitude.coeffs();
        pred_mu.segment<3>(7) = m_camera_state.linear_velocity;
        pred_mu.segment<3>(10) = m_camera_state.angular_velocity;

        for(int i=0; i<num_landmarks; i++)
        {
            pred_mu.segment<3>(13+3*i) = m_landmarks[i].position;
        }
    }

    {
        Eigen::VectorXi reservation(dim);
        reservation.fill(1.0);
        reservation.head<7>() << 2, 2, 2, 7, 7, 7, 7;

        // J est la jacobienne de la fonction de transition vers le nouvel état.
        Eigen::SparseMatrix<double> J(dim, dim);
        J.reserve(reservation);

        J.insert(0,0) = 1.0;
        J.insert(0,7) = dt;
        J.insert(1,1) = 1.0;
        J.insert(1,8) = dt;
        J.insert(2,2) = 1.0;
        J.insert(2,9) = dt;

        // TODO: finir de remplir J.

        J.makeCompressed();

        // Q est la matrice de covariance du bruit sur la prédiction du nouvel état.
        Eigen::SparseMatrix<double> Q(dim, dim);

        // TODO: fill matrix Q.

        Q.makeCompressed();

        pred_sigma = J * m_state_covariance * J.transpose() + Q;
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


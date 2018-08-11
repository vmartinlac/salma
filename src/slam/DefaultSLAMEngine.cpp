#include <opencv2/calib3d.hpp>
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
    m_mode = MODE_INITIALIZATION;

    retrieveParameters();

    if( m_camera == nullptr ) throw std::runtime_error("Internal error");

    m_camera_state.translation.setZero();
    m_camera_state.rotation.setIdentity();
    m_landmarks.clear();
    m_state_covariance.resize(0, 0);
    m_candidate_landmarks.clear();

    m_camera->open();

    while( isInterruptionRequested() == false )
    {
        Image image;
        m_camera->read(image);

        if(image.isValid())
        {
            switch(m_mode)
            {
            case MODE_INITIALIZATION:
                processImageInitializing(image);
                break;
            case MODE_SLAM:
                processImageSLAM(image);
                break;
            case MODE_LOST:
            default:
                processImageLost(image);
                break;
            }
        }
        else
        {
            QThread::yieldCurrentThread();
        }
    }

    m_camera->close();
}

void DefaultSLAMEngine::processImageInitializing(Image& image)
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

        m_camera_state.translation <<
            translation.at<float>(0, 0),
            translation.at<float>(1, 0),
            translation.at<float>(2, 0);

        Eigen::Vector3d rodrigues <<
            rodrigues_rotation.at<float>(0, 0),
            rodrigues_rotation.at<float>(1, 0),
            rodrigues_rotation.at<float>(2, 0);

        const double norm = rodrigues.norm();
        const double cos_half_theta = cos(0.5*norm);
        const double sin_half_theta = sin(0.5*norm);

        m_camera_state.rotation << 
            rodrigues(0)*sin_half_theta/norm,
            rodrigues(1)*sin_half_theta/norm,
            rodrigues(2)*sin_half_theta/norm,
            cos_half_theta ;

        // create first landmarks.

        // TODO

        // set mode to SLAM.

        m_mode = MODE_SLAM;
    }
}

void DefaultSLAMEngine::processImageSLAM(Image& image)
{
}

void DefaultSLAMEngine::processImageLost(Image& image)
{
}

SLAMEngine* SLAMEngine::create(Camera* camera)
{
    return new DefaultSLAMEngine(camera);
}


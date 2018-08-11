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

        Eigen::Vector3d rodrigues;
        rodrigues <<
            rodrigues_rotation.at<float>(0, 0),
            rodrigues_rotation.at<float>(1, 0),
            rodrigues_rotation.at<float>(2, 0);

        const double norm = rodrigues.norm();
        const double cos_half_theta = cos(0.5*norm);
        const double sin_half_theta = sin(0.5*norm);

        m_camera_state.rotation.coeffs() << 
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
                m_landmarks.size(),
                m_landmarks.size() );

            m_state_covariance.setZero();
            m_state_covariance.diagonal().fill(sigma*sigma);

            m_mode = MODE_SLAM;
            std::cout << "Initialized ! Switching to SLAM mode." << std::endl;
        }
        else
        {
            m_landmarks.clear();
        }
    }
}

void DefaultSLAMEngine::processImageSLAM(Image& image)
{
}

void DefaultSLAMEngine::processImageLost(Image& image)
{
}

bool DefaultSLAMEngine::extractPatch(cv::Mat& image, float x, float y, cv::Mat& patch)
{
    const int radius = m_parameters.patch_size/2;

    const int X = (int) cvRound(x);
    const int Y = (int) cvRound(y);

    // TODO !

    return false;
}

SLAMEngine* SLAMEngine::create(Camera* camera)
{
    return new DefaultSLAMEngine(camera);
}


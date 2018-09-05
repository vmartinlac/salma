
#pragma once

#include <Eigen/Eigen>
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include "SLAMEngine.h"
#include "Camera.h"
#include "Image.h"
#include "Tracker.h"

class SLAMEngineImpl : public SLAMEngine
{
public:

    SLAMEngineImpl();

    virtual void run();

protected:

    enum Mode
    {
        MODE_INIT,
        MODE_SLAM,
        MODE_DEAD
    };

    struct CameraState
    {
        Eigen::Vector3d position;
        Eigen::Quaterniond attitude;
        Eigen::Vector3d linear_velocity;
        Eigen::Vector3d angular_velocity;
    };

    struct Landmark
    {
        Eigen::Vector3d position;
        cv::Mat descriptor_or_template;
        int num_failed_detections;
        int num_successful_detections;
        int last_seen_frame;
    };

    struct CandidateLandmark
    {
        cv::Mat descriptor;
        Eigen::Vector3d origin;
        Eigen::Vector3d direction;
    };

protected:

    void setup();

    void processImageInit();
    void processImageSLAM();
    void processImageDead();

    void writeOutput();

    void EKFPredict(Eigen::VectorXd& mu, Eigen::MatrixXd& sigma);
    void EKFUpdate(Eigen::VectorXd& mu, Eigen::MatrixXd& sigma);

    void retrieveBelief(Eigen::VectorXd& mu, Eigen::MatrixXd& sigma);
    void storeBelief(Eigen::VectorXd& mu, Eigen::MatrixXd& sigma);

    void computeResiduals(
        const Eigen::VectorXd& state_mu,
        const Eigen::MatrixXd& state_sigma,
        std::vector<int>& selection,
        Eigen::VectorXd& h,
        Eigen::SparseMatrix<double>& J,
        Eigen::VectorXd& residuals);

    void matchWithTemplates(
        const Eigen::VectorXd& state_mu,
        const Eigen::MatrixXd& state_sigma,
        const std::vector<int>& selection,
        const Eigen::VectorXd& h,
        const Eigen::SparseMatrix<double>& J,
        Eigen::VectorXd& residuals,
        std::vector<bool> found);

    void matchWithDescriptors(
        const Eigen::VectorXd& state_mu,
        const Eigen::MatrixXd& state_sigma,
        const std::vector<int>& selection,
        const Eigen::VectorXd& h,
        const Eigen::SparseMatrix<double>& J,
        Eigen::VectorXd& residuals,
        std::vector<bool> found);
        

protected:

    // TODO: put this into parameters.
    enum
    {
        TRACKING_TEMPLATE=0,
        TRACKING_DESCRIPTOR=1
    };
    int m_tracking_method;
    double m_measurement_standard_deviation;
    //

    cv::Mat m_calibration_matrix;
    cv::Mat m_distortion_coefficients;

    Mode m_mode;
    Image m_current_image;
    Image m_previous_image;
    double m_time_last_frame;
    int m_frame_id;

    CameraState m_camera_state;
    std::vector<Landmark> m_landmarks;
    Eigen::MatrixXd m_state_covariance;
    std::vector<CandidateLandmark> m_candidate_landmarks;

    target::Tracker m_tracker;
    cv::Ptr<cv::Feature2D> m_feature;
};



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
        //MODE_TT,
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
        cv::Mat descriptor;
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
    //void processImageTT();
    void processImageSLAM();
    void processImageDead();

    void writeOutput();

    void EKFPredict(Eigen::VectorXd& pred_mu, Eigen::MatrixXd& pred_sigma);
    void EKFUpdate(Eigen::VectorXd& mu, Eigen::MatrixXd& sigma);
    void saveState(Eigen::VectorXd& mu, Eigen::MatrixXd& sigma);

protected:

    cv::Mat m_calibration_matrix;
    cv::Mat m_distortion_coefficients;

    Mode m_mode;
    Image m_current_image;
    double m_time_last_frame;
    int m_frame_id;

    CameraState m_camera_state;
    std::vector<Landmark> m_landmarks;
    Eigen::MatrixXd m_state_covariance;
    std::vector<CandidateLandmark> m_candidate_landmarks;

    target::Tracker m_tracker;
    cv::Ptr<cv::Feature2D> m_feature;
};


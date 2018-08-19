
#pragma once

#include <Eigen/Eigen>
#include <vector>
#include <opencv2/opencv.hpp>
#include "SLAMEngine.h"
#include "Camera.h"
#include "Image.h"

class DefaultSLAMEngine : public SLAMEngine
{
public:

    DefaultSLAMEngine();

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
        cv::Mat patch;
        int num_failed_detections;
        int num_successful_detections;
        double last_successful_detection_time;
    };

    struct CandidateLandmark
    {
        cv::Mat patch;
        Eigen::Vector3d origin;
        Eigen::Vector3d direction;
    };

protected:

    void processImageInit();
    void processImageSLAM();
    void processImageDead();

    bool extractPatch( const cv::Point2i& point, cv::Mat& patch );
    bool findPatch(
        const cv::Mat& patch,
        const cv::Rect& area,
        cv::Point2i& found_coords );
    bool comparePatches(const cv::Mat& P1, const cv::Mat& P2);

    void EKFPredict(Eigen::VectorXd& pred_mu, Eigen::MatrixXd& pred_sigma);
    void EKFUpdate(Eigen::VectorXd& mu, Eigen::MatrixXd& sigma);
    void saveState(Eigen::VectorXd& mu, Eigen::MatrixXd& sigma);

protected:

    Mode m_mode;
    cv::Mat m_calibration_matrix;
    cv::Mat m_distortion_coefficients;
    CameraState m_camera_state;
    std::vector<Landmark> m_landmarks;
    Eigen::MatrixXd m_state_covariance;
    std::vector<CandidateLandmark> m_candidate_landmarks;
    Image m_current_image;
    std::vector<cv::Point2i> m_current_corners;
    double m_time_last_frame;
};


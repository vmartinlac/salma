
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

    DefaultSLAMEngine(Camera* camera);

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

    struct Parameters
    {
        cv::Mat calibration_matrix;
        cv::Mat distortion_coefficients;
        double target_unit_length;
        int patch_size;
        int num_depth_hypotheses;
        double min_distance_to_camera;
        cv::Rect image_viewport;
    };

    struct Landmark
    {
        Eigen::Vector3d position;
        cv::Mat patch;
        int num_detection_failures;
    };

    struct CandidateLandmark
    {
        cv::Mat patch;
        Eigen::Vector3d origin;
        Eigen::Vector3d direction;
        std::vector<float> depth_hypotheses;
    };

protected:

    void retrieveParameters();

    void processImageInit(Image& image);
    void processImageSLAM(Image& image);
    void processImageDead(Image& image);

    bool extractPatch(cv::Mat& image, float x, float y, cv::Mat& patch);
    bool findPatch(
        const cv::Mat& image,
        const cv::Mat& patch,
        double box_center_u,
        double box_center_v,
        double box_radius_u,
        double box_radius_v,
        double& found_u,
        double& found_v);

    void EKFPredict(double dt, Eigen::VectorXd& pred_mu, Eigen::MatrixXd& pred_sigma);
    void EKFUpdate(Image& image, Eigen::VectorXd& mu, Eigen::MatrixXd& sigma);
    void saveState(Eigen::VectorXd& mu, Eigen::MatrixXd& sigma);

protected:

    Mode m_mode;
    Parameters m_parameters;
    Camera* m_camera;
    CameraState m_camera_state;
    double m_time_last_frame;
    std::vector<Landmark> m_landmarks;
    Eigen::MatrixXd m_state_covariance;
    std::vector<CandidateLandmark> m_candidate_landmarks;
    cv::Rect m_viewport;
};


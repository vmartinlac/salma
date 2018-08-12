
#pragma once

#include <Eigen/Eigen>
#include <vector>
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
        MODE_INITIALIZATION,
        MODE_SLAM,
        MODE_LOST
    };

    struct CameraState
    {
        Eigen::Vector3d translation;
        Eigen::Quaterniond rotation;
        Eigen::Vector3d linear_momentum;
        Eigen::Vector3d angular_momentum;
    };

    struct Parameters
    {
        cv::Mat calibration_matrix;
        cv::Mat distortion_coefficients;
        float target_unit_length;
        int patch_size;
        int num_depth_hypotheses;
    };

    struct Landmark
    {
        Eigen::Vector3d position;
        cv::Mat patch;
    };

    struct CandidateLandmark
    {
        cv::Mat patch;
        Eigen::Vector3d origin;
        Eigen::Vector3d direction;
        std::vector<float> depth_hypotheses;
    };

protected:

    bool extractPatch(cv::Mat& image, float x, float y, cv::Mat& patch);
    void retrieveParameters();
    void processImageInitializing(Image& image);
    void processImageSLAM(Image& image);
    void processImageLost(Image& image);

protected:

    Mode m_mode;
    Parameters m_parameters;
    Camera* m_camera;
    CameraState m_camera_state;
    std::vector<Landmark> m_landmarks;
    Eigen::MatrixXd m_state_covariance;
    std::vector<CandidateLandmark> m_candidate_landmarks;
};


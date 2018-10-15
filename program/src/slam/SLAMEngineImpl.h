
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
        MODE_TARGET_NOT_FOUND,
        MODE_LOST,
        MODE_TRACKING
    };

    struct CameraState
    {
        Eigen::Vector3d position;
        Eigen::Quaterniond attitude;
        Eigen::Vector3d linear_velocity;
        Eigen::Vector3d angular_velocity;
    };

    typedef Eigen::Matrix<double,13,1> BeliefMean;
    typedef Eigen::Matrix<double,13,13> BeliefCovariance;

protected:

    void setup();

    void processImage();

    void localizationPnP();

    void localizationEKF();

    void writeOutput();

    static void compute_f(
        const BeliefMean& X,
        double dt,
        BeliefMean& f,
        Eigen::SparseMatrix<double>& J);

    static void compute_h(
        const BeliefMean& X,
        const std::vector<cv::Point3f>& object_points,
        const cv::Mat& camera_matrix,
        const cv::Mat& distortion_coefficients,
        Eigen::VectorXd& h,
        Eigen::SparseMatrix<double>& J);

    static void computeJacobianOfWorld2CameraTransformation(
        const BeliefMean& X,
        const cv::Point3f& object_point,
        Eigen::Matrix<double, 3, 10>& J);

    static void convertPose(
        const cv::Mat& rodrigues,
        const cv::Mat& t,
        Eigen::Quaterniond& attitude,
        Eigen::Vector3d& position);

protected:

    // TODO: put the following parameter into appropriate data structure.
    double m_measurement_standard_deviation;

    cv::Mat m_calibration_matrix;
    cv::Mat m_distortion_coefficients;

    target::Tracker m_tracker;

    Mode m_mode;
    Image m_current_image;
    double m_time_last_frame;
    int m_frame_id;

    CameraState m_camera_state;
    BeliefMean m_belief_mean;
    BeliefCovariance m_belief_covariance;
};


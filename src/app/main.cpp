#include <QApplication>
#include <array>
#include <iostream>
#include "Camera.h"
#include "Image.h"
#include <QThread>
#include <Eigen/Eigen>
#include "utils.h"
#include "target.h"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

/*

struct SLAMEngineParameters
{
    Eigen::Matrix3d calibration_matrix;
    int patch_size;
    int num_depth_hypotheses;
};

class SLAMEngine : public QThread
{
public:

    SLAMEngine(const SLAMEngineParameters& parameters, Camera* camera) : m_parameters(parameters), m_camera(m_camera)
    {
        ;
    }

    virtual void run()
    {
        m_camera->start();

        while( isInterruptionRequested() == false )
        {
        }

        m_camera->stop();
    }

protected:

    typedef Eigen::Matrix<double, 12, 1> CameraState;

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

    SLAMEngineParameters m_parameters;
    Camera* m_camera;
    CameraState m_camera_state;
    std::vector<Landmark> m_landmarks;
    Eigen::MatrixXd m_state_covariance;
    std::vector<CandidateLandmark> m_candidate_landmarks;
};

*/

int main(int num_args, char** args)
{
    QApplication app(num_args, args);

    cv::Mat image;
    image = cv::imread("/home/victor/developpement/slam/data/photographies_mire/a.jpg");
    std::cout << image.cols << "*" << image.rows << std::endl;
    //cv::resize(image, image, cv::Size(1024, 768));
    //cv::resize(image, image, cv::Size(640, 480));

    target::Detector d;
    cv::Mat samples;
    d.run(image, samples);

    return 0;
}


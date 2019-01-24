#include <Eigen/Eigen>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/cudastereo.hpp>
#include <opencv2/stereo.hpp>
#include <opencv2/core/cuda.hpp>
#include "StereoMatcher.h"

int main(int num_args, char** args)
{
    cv::Mat left = cv::imread("/home/victor/stereo_images/tsukuba/scene1.row3.col1.bmp");
    cv::Mat right = cv::imread("/home/victor/stereo_images/tsukuba/scene1.row3.col2.bmp");

    if(left.data == nullptr || right.data == nullptr) throw std::runtime_error("Could not load images!");

    cv::cuda::GpuMat d_left;
    cv::cuda::GpuMat d_right;
    d_left.upload(left);
    d_right.upload(right);

    //auto m = cv::StereoSGBM::create();
    //auto m = cv::cuda::createStereoBeliefPropagation();
    auto m = StereoMatcher::create();

    cv::cuda::GpuMat d_disp;
    m->compute(d_left, d_right, d_disp);

    cv::Mat disp;
    d_disp.download(disp);

    cv::imwrite("rien.bmp", disp);

    return 0;
}


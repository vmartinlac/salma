#include <iostream>
#include <opencv2/core/cuda.hpp>
#include <opencv2/imgcodecs.hpp>
#include "StereoMatcherImpl.h"

__global__ void hello(
    cv::cuda::PtrStepSz<uchar3> left,
    cv::cuda::PtrStepSz<uchar3> right,
    cv::cuda::PtrStepSz<uchar3> image)
{
    const int x = blockDim.x * blockIdx.x + threadIdx.x;
    const int y = blockDim.y * blockIdx.y + threadIdx.y;

    if(x < image.cols && y < image.rows)
    {
        uchar3 l = left(y,x);
        uchar3 r = right(y,x);
    }
}


StereoMatcherImpl::StereoMatcherImpl()
{
    mNumDisparities = 16;
}

void StereoMatcherImpl::compute(cv::InputArray left, cv::InputArray right, cv::OutputArray disparity)
{
    cv::cuda::GpuMat d_left = left.getGpuMat();
    cv::cuda::GpuMat d_right = right.getGpuMat();
    cv::cuda::GpuMat& d_disp = disparity.getGpuMatRef();

    const int width = d_left.cols;
    const int height = d_left.rows;

    if( width != d_right.cols || height != d_right.rows ) throw std::runtime_error("left and right images should be the same size!");

    mBlockWidth = 16;
    mBlockHeight = 16;
    mGridWidth = (width + mBlockWidth - 1) / mBlockWidth;
    mGridHeight = (height + mBlockHeight - 1) / mBlockHeight;

    d_disp.create( width, height, CV_8UC3 );

    hello<<< dim3(mGridWidth,mGridHeight), dim3(mBlockWidth,mBlockHeight) >>>(d_left, d_right, d_disp);

    cudaDeviceSynchronize();
}

StereoMatcher* StereoMatcher::create()
{
    return new StereoMatcherImpl();
}

int StereoMatcherImpl::getMinDisparity() const
{
    return 0;
}


void StereoMatcherImpl::setMinDisparity(int minDisparity)
{
    // ignored
}


int StereoMatcherImpl::getNumDisparities() const
{
    return mNumDisparities;
}


void StereoMatcherImpl::setNumDisparities(int numDisparities)
{
    mNumDisparities = numDisparities;
}


int StereoMatcherImpl::getBlockSize() const
{
    return 0;
}


void StereoMatcherImpl::setBlockSize(int blockSize)
{
    // ignored
}


int StereoMatcherImpl::getSpeckleWindowSize() const
{
    return 0;
}


void StereoMatcherImpl::setSpeckleWindowSize(int speckleWindowSize)
{
    // ignored
}


int StereoMatcherImpl::getSpeckleRange() const
{
    return 0;
}


void StereoMatcherImpl::setSpeckleRange(int speckleRange)
{
    // ignored
}


int StereoMatcherImpl::getDisp12MaxDiff() const
{
    return 0;
}


void StereoMatcherImpl::setDisp12MaxDiff(int disp12MaxDiff)
{
    // ignored
}



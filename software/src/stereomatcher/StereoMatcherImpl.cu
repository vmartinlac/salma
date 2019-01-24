#include <iostream>
#include <vector>
#include <opencv2/core/cuda.hpp>
#include <opencv2/core/cuda_stream_accessor.hpp>
#include <opencv2/imgcodecs.hpp>
#include "StereoMatcherImpl.h"

__global__ void hello(cv::cuda::PtrStepSz<uchar3> image)
{
    const int x = blockDim.x * blockIdx.x + threadIdx.x;
    const int y = blockDim.y * blockIdx.y + threadIdx.y;

    if(x < image.cols && y < image.rows)
    {
        x+y;
    }
}


StereoMatcherImpl::StereoMatcherImpl()
{
    mNumDisparities = 16;
}

void StereoMatcherImpl::compute(cv::InputArray left, cv::InputArray right, cv::OutputArray disparity)
{
    cv::cuda::GpuMat d_occlusion_left;
    cv::cuda::GpuMat d_occlusion_right;

    cv::cuda::GpuMat d_disparity_left;
    cv::cuda::GpuMat d_disparity_right;

    cv::cuda::Stream stream_left;
    cv::cuda::Stream stream_right;

    cv::cuda::GpuMat d_image_left = left.getGpuMat();
    cv::cuda::GpuMat d_image_right = right.getGpuMat();

    if( d_image_left.size() != d_image_right.size() ) throw std::runtime_error("left and right images should be the same size!");

    const cv::Size size = d_image_left.size();

    d_occlusion_left.create( size, CV_8UC1 );
    d_occlusion_right.create( size, CV_8UC1 );

    d_disparity_left.create( size, CV_16SC1 );
    d_disparity_right.create( size, CV_16SC1 );

    const dim3 block_dim( 16, 16 );
    const dim3 grid_dim( (size.width + block_dim.x - 1) / block_dim.x, (size.height + block_dim.y - 1) / block_dim.y );

    const cudaStream_t cuda_stream_left = cv::cuda::StreamAccessor::getStream(stream_left);
    const cudaStream_t cuda_stream_right = cv::cuda::StreamAccessor::getStream(stream_right);

    const int num_iterations_for_occlusion_belief_propagation = 5;
    const int num_iterations_for_disparity_belief_propagation = 5;
    const int num_iterations_for_fixed_point = 3;

    std::vector<cv::cuda::Event> occlusion_events_left(num_iterations_for_fixed_point);
    std::vector<cv::cuda::Event> occlusion_events_right(num_iterations_for_fixed_point);
    std::vector<cv::cuda::Event> disparity_events_left(num_iterations_for_fixed_point);
    std::vector<cv::cuda::Event> disparity_events_right(num_iterations_for_fixed_point);

    for(int i=0; i<num_iterations_for_fixed_point; i++)
    {
        for(int j=0; j<num_iterations_for_occlusion_belief_propagation; j++)
        {
            hello<<<grid_dim, block_dim, 0, cuda_stream_left>>>(d_image_left);
            hello<<<grid_dim, block_dim, 0, cuda_stream_right>>>(d_image_right);
        }

        occlusion_events_left[i].record(stream_left);
        occlusion_events_right[i].record(stream_right);

        stream_left.waitEvent( occlusion_events_right[i] );
        stream_right.waitEvent( occlusion_events_left[i] );

        for(int j=0; j<num_iterations_for_disparity_belief_propagation; j++)
        {
            hello<<<grid_dim, block_dim, 0, cuda_stream_left>>>(d_image_left);
            hello<<<grid_dim, block_dim, 0, cuda_stream_right>>>(d_image_right);
        }

        occlusion_events_left[i].record(stream_left);
        occlusion_events_right[i].record(stream_right);

        stream_left.waitEvent( disparity_events_right[i] );
        stream_right.waitEvent( disparity_events_left[i] );
    }

    stream_left.waitForCompletion();
    stream_right.waitForCompletion();

    // TODO retrieve resulting disparity.
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



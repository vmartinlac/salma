#include <iostream>
#include <vector>
#include <opencv2/core/cuda.hpp>
#include <opencv2/core/cuda/common.hpp>
#include <opencv2/core/cuda_stream_accessor.hpp>
#include <opencv2/imgcodecs.hpp>
#include "StereoMatcherImpl.h"

__constant__ int cte_margin = 0;
__constant__ int cte_num_disparities = 0;
__constant__ int cte_width = 0;
__constant__ int cte_height = 0;
__constant__ int cte_neighbor_dx[4] = { 1, 0, -1, 0 };
__constant__ int cte_neighbor_dy[4] = { 0, 1, 0, -1 };

__device__ size_t occlusion_message_index(int xfrom, int yfrom, int neighbor, int msg)
{
    return 4*2*cte_width*yfrom + 4*2*xfrom + 2*neighbor + msg;
}

__device__ size_t disparity_message_index(int xfrom, int yfrom, int neighbor, int msg)
{
    return 4*cte_num_disparities*cte_width*yfrom + 4*cte_num_disparities*xfrom + cte_num_disparities*neighbor + msg;
}

__device__ bool isInROI(int x, int y)
{
    return ( cte_margin <= x && x < cte_width - cte_margin && cte_margin <= y && y < cte_height - cte_margin );
}

__global__ void kernel_initialize_disparity_and_occlusion(
    cv::cuda::PtrStep<short1> disparity_left,
    cv::cuda::PtrStep<short1> disparity_right,
    cv::cuda::PtrStep<uchar1> occlusion_left,
    cv::cuda::PtrStep<uchar1> occlusion_right)
{
    const int x = blockDim.x * blockIdx.x + threadIdx.x;
    const int y = blockDim.y * blockIdx.y + threadIdx.y;

    if( x < cte_width && y < cte_height )
    {
        disparity_left(y,x) = make_short1(0);
        disparity_right(y,x) = make_short1(0);
        occlusion_left(y,x) = make_uchar1(0);
        occlusion_right(y,x) = make_uchar1(0);
    }
}

__global__ void kernel_clear_warp_and_messages(
    cv::cuda::PtrStep<uchar1> warp_left,
    cv::cuda::PtrStep<uchar1> warp_right,
    float* msg_occlusion_left,
    float* msg_occlusion_right,
    float* msg_disparity_left,
    float* msg_disparity_right)
{
    const int x = blockDim.x * blockIdx.x + threadIdx.x;
    const int y = blockDim.y * blockIdx.y + threadIdx.y;

    if( x < cte_width && y < cte_height)
    {
        warp_left(y,x) = make_uchar1(1);
        warp_right(y,x) = make_uchar1(1);

        for(int i=0; i<4; i++)
        {
            for(int j=0; j<2; j++)
            {
                msg_occlusion_left[occlusion_message_index(x, y, i, j)] = 0.0;
                msg_occlusion_right[occlusion_message_index(x, y, i, j)] = 0.0;
            }

            for(int j=0; j<cte_num_disparities; j++)
            {
                msg_disparity_left[disparity_message_index(x, y, i, j)] = 0.0;
                msg_disparity_right[disparity_message_index(x, y, i, j)] = 0.0;
            }
        }
    }
}

__global__ void kernel_compute_warp(
    cv::cuda::PtrStep<short1> disparity_left,
    cv::cuda::PtrStep<short1> disparity_right,
    cv::cuda::PtrStep<uchar1> occlusion_left,
    cv::cuda::PtrStep<uchar1> occlusion_right,
    cv::cuda::PtrStep<uchar1> warp_left,
    cv::cuda::PtrStep<uchar1> warp_right)
{
    const int x = blockDim.x * blockIdx.x + threadIdx.x;
    const int y = blockDim.y * blockIdx.y + threadIdx.y;

    if( isInROI(x,y) )
    {
        // compute left warp.

        if( occlusion_right(y,x).x == 0 )
        {
            const int left_x = x + disparity_right(y,x).x;

            if( cte_margin <= left_x && left_x < cte_width - cte_margin )
            {
                warp_left(y,left_x) = make_uchar1(0);
            }
        }

        // compute right warp.

        if( occlusion_left(y,x).x == 0 )
        {
            const int right_x = x + disparity_left(y,x).x;

            if( cte_margin <= right_x && right_x < cte_width - cte_margin )
            {
                warp_right(y,right_x) = make_uchar1(0);
            }
        }
    }
}

__global__ void kernel_occlusion_iteration(
    cv::cuda::PtrStep<uchar3> image_left,
    cv::cuda::PtrStep<uchar3> image_right,
    cv::cuda::PtrStep<uchar1> warp_left,
    cv::cuda::PtrStep<uchar1> warp_right,
    cv::cuda::PtrStep<short1> disparity_left,
    cv::cuda::PtrStep<short1> disparity_right,
    float* occlusion_messages_left_pre,
    float* occlusion_messages_right_pre,
    float* occlusion_messages_left_post,
    float* occlusion_messages_right_post)
{
    const int x = blockDim.x * blockIdx.x + threadIdx.x;
    const int y = blockDim.y * blockIdx.y + threadIdx.y;

    if( isInROI(x,y) )
    {
        for(int i=0; i<4; i++)
        {
            const int xo = x + cte_neighbor_dx[i];
            const int yo = y + cte_neighbor_dy[i];

            if( isInROI(xo, yo) )
            {
                for(int fo=0; fo<2; fo++)
                {
                    float best_value_left = 0.0f;
                    float best_value_right = 0.0f;

                    for(int f=0; f<2; f++)
                    {
                        float value_left = 0.0f;
                        float value_right = 0.0f;

                        // TODO

                        if(f == 0 || value_left < best_value_left)
                        {
                            best_value_left = value_left;
                        }

                        if(f == 0 || value_right < best_value_right)
                        {
                            best_value_right = value_right;
                        }
                    }

                    occlusion_messages_left_post[occlusion_message_index(x, y, i, fo)] = best_value_left;
                    occlusion_messages_right_post[occlusion_message_index(x, y, i, fo)] = best_value_right;
                }
            }
        }
    }
}

__global__ void kernel_occlusion_update()
{
}

__global__ void kernel_disparity_iteration()
{
}

__global__ void kernel_disparity_update()
{
}

StereoMatcherImpl::StereoMatcherImpl()
{
    mNumDisparities = 16;
    mMargin = 1;
}

void StereoMatcherImpl::compute(cv::InputArray left, cv::InputArray right, cv::OutputArray disparity)
{
    const int num_iterations_for_occlusion_belief_propagation = 8;
    const int num_iterations_for_disparity_belief_propagation = 8;
    const int num_iterations_for_fixed_point = 4;

    cv::cuda::GpuMat d_image_left = left.getGpuMat();
    cv::cuda::GpuMat d_image_right = right.getGpuMat();

    if( d_image_left.size() != d_image_right.size() ) throw std::runtime_error("left and right images should be the same size!");

    const cv::Size2i size = d_image_left.size();

    if( size.width <= 2*mMargin || size.height <= 2*mMargin ) throw std::runtime_error("image is too small.");

    cv::cuda::GpuMat d_occlusion_left;
    d_occlusion_left.create( size, CV_8UC1 );

    cv::cuda::GpuMat d_occlusion_right;
    d_occlusion_right.create( size, CV_8UC1 );

    cv::cuda::GpuMat d_disparity_left;
    d_disparity_left.create( size, CV_16SC1 );

    cv::cuda::GpuMat d_disparity_right;
    d_disparity_right.create( size, CV_16SC1 );

    cv::cuda::GpuMat d_warp_left;
    d_warp_left.create( size, CV_8UC1 );

    cv::cuda::GpuMat d_warp_right;
    d_warp_right.create( size, CV_8UC1 );

    float* d_occlusion_messages_left[2];
    float* d_occlusion_messages_right[2];
    float* d_disparity_messages_left[2];
    float* d_disparity_messages_right[2];
    cudaSafeCall( cudaMalloc(&d_occlusion_messages_left[0], size.width*size.height*2*4*sizeof(float)) );
    cudaSafeCall( cudaMalloc(&d_occlusion_messages_left[1], size.width*size.height*2*4*sizeof(float)) );
    cudaSafeCall( cudaMalloc(&d_occlusion_messages_right[0], size.width*size.height*2*4*sizeof(float)) );
    cudaSafeCall( cudaMalloc(&d_occlusion_messages_right[1], size.width*size.height*2*4*sizeof(float)) );
    cudaSafeCall( cudaMalloc(&d_disparity_messages_left[0], size.width*size.height*mNumDisparities*4*sizeof(float)) );
    cudaSafeCall( cudaMalloc(&d_disparity_messages_left[1], size.width*size.height*mNumDisparities*4*sizeof(float)) );
    cudaSafeCall( cudaMalloc(&d_disparity_messages_right[0], size.width*size.height*mNumDisparities*4*sizeof(float)) );
    cudaSafeCall( cudaMalloc(&d_disparity_messages_right[1], size.width*size.height*mNumDisparities*4*sizeof(float)) );

    const dim3 block_dim( 16, 16 );
    const dim3 grid_dim( (size.width + block_dim.x - 1) / block_dim.x, (size.height + block_dim.y - 1) / block_dim.y );

    cv::cuda::Stream stream;
    const cudaStream_t stream_id = cv::cuda::StreamAccessor::getStream(stream);

    cudaSafeCall( cudaMemcpyToSymbol(cte_margin, &mMargin, sizeof(int)) );
    cudaSafeCall( cudaMemcpyToSymbol(cte_num_disparities, &mNumDisparities, sizeof(int)) );
    cudaSafeCall( cudaMemcpyToSymbol(cte_width, &size.width, sizeof(int)) );
    cudaSafeCall( cudaMemcpyToSymbol(cte_height, &size.height, sizeof(int)) );

    kernel_initialize_disparity_and_occlusion<<<grid_dim, block_dim, 0, stream_id>>>(
        d_disparity_left,
        d_disparity_right,
        d_occlusion_left,
        d_occlusion_right);

    for(int i=0; i<num_iterations_for_fixed_point; i++)
    {
        kernel_clear_warp_and_messages<<<grid_dim, block_dim, 0, stream_id>>>(
            d_warp_left,
            d_warp_right,
            d_occlusion_messages_left[0],
            d_occlusion_messages_right[0],
            d_disparity_messages_left[0],
            d_disparity_messages_right[0]);

        kernel_compute_warp<<<grid_dim, block_dim, 0, stream_id>>>(
            d_disparity_left,
            d_disparity_right,
            d_occlusion_left,
            d_occlusion_right,
            d_warp_left,
            d_warp_right);

        for(int j=0; j<num_iterations_for_occlusion_belief_propagation; j++)
        {
            kernel_occlusion_iteration<<<grid_dim, block_dim, 0, stream_id>>>(
                d_image_left,
                d_image_right,
                d_warp_left,
                d_warp_right,
                d_disparity_left,
                d_disparity_right,
                d_occlusion_messages_left[0],
                d_occlusion_messages_right[0],
                d_occlusion_messages_left[1],
                d_occlusion_messages_right[1]);

            std::swap( d_occlusion_messages_left[0], d_occlusion_messages_left[1] );
            std::swap( d_occlusion_messages_right[0], d_occlusion_messages_right[1] );
        }

        kernel_occlusion_update<<<grid_dim, block_dim, 0, stream_id>>>();

        for(int j=0; j<num_iterations_for_disparity_belief_propagation; j++)
        {
            kernel_disparity_iteration<<<grid_dim, block_dim, 0, stream_id>>>();
                /*
                d_image_left,
                d_image_right,
                d_warp_left,
                d_warp_right,
                d_disparity_left,
                d_disparity_right,
                d_disparity_messages_left[0],
                d_disparity_messages_right[0],
                d_disparity_messages_left[1],
                d_disparity_messages_right[1]);
                */

            std::swap( d_disparity_messages_left[0], d_disparity_messages_left[1] );
            std::swap( d_disparity_messages_right[0], d_disparity_messages_right[1] );
        }

        kernel_disparity_update<<<grid_dim, block_dim, 0, stream_id>>>();
    }

    stream.waitForCompletion();

    disparity.getGpuMatRef() = d_disparity_left;

    cudaSafeCall( cudaFree(d_occlusion_messages_left[0]) );
    cudaSafeCall( cudaFree(d_occlusion_messages_left[1]) );
    cudaSafeCall( cudaFree(d_occlusion_messages_right[0]) );
    cudaSafeCall( cudaFree(d_occlusion_messages_right[1]) );
    cudaSafeCall( cudaFree(d_disparity_messages_left[0]) );
    cudaSafeCall( cudaFree(d_disparity_messages_left[1]) );
    cudaSafeCall( cudaFree(d_disparity_messages_right[0]) );
    cudaSafeCall( cudaFree(d_disparity_messages_right[1]) );
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



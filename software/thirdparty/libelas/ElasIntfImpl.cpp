#include "ElasIntfImpl.h"

ElasIntfImpl::ElasIntfImpl()
{
}

void ElasIntfImpl::compute( cv::InputArray left, cv::InputArray right, cv::OutputArray disparity )
{
    if( bool(mElas) == false )
    {
        Elas::parameters params(Elas::ROBOTICS);

        mElas.reset(new Elas(params));
    }

    cv::Mat mat_left = left.getMat();
    cv::Mat mat_right = right.getMat();

    if( mat_left.size() != mat_right.size() ) throw std::runtime_error("image sizes differ!");
    if( mat_left.type() != CV_8UC1 || mat_right.type() != CV_8UC1 ) throw std::runtime_error("incorrect type!");
    // TODO: check same bytes par line.

    cv::Mat mat_left_disp( mat_left.size(), CV_32F );
    cv::Mat mat_right_disp( mat_right.size(), CV_32F );

    int32_t dims[3];
    dims[0] = mat_left.cols;
    dims[1] = mat_left.rows;
    //dims[2] = mat_

    mElas->process(
        reinterpret_cast<uint8_t*>( mat_left.ptr() ),
        reinterpret_cast<uint8_t*>( mat_right.ptr() ),
        reinterpret_cast<float*>( mat_left_disp.ptr() ),
        reinterpret_cast<float*>( mat_right_disp.ptr() ),
        dims);
}

int ElasIntfImpl::getMinDisparity() const
{
    throw std::runtime_error("not implemented");
}


void ElasIntfImpl::setMinDisparity(int minDisparity)
{
    throw std::runtime_error("not implemented");
}


int ElasIntfImpl::getNumDisparities() const
{
    throw std::runtime_error("not implemented");
}


void ElasIntfImpl::setNumDisparities(int numDisparities)
{
    throw std::runtime_error("not implemented");
}


int ElasIntfImpl::getBlockSize() const
{
    throw std::runtime_error("not implemented");
}


void ElasIntfImpl::setBlockSize(int blockSize)
{
    throw std::runtime_error("not implemented");
}


int ElasIntfImpl::getSpeckleWindowSize() const
{
    throw std::runtime_error("not implemented");
}


void ElasIntfImpl::setSpeckleWindowSize(int speckleWindowSize)
{
    throw std::runtime_error("not implemented");
}


int ElasIntfImpl::getSpeckleRange() const
{
    throw std::runtime_error("not implemented");
}


void ElasIntfImpl::setSpeckleRange(int speckleRange)
{
    throw std::runtime_error("not implemented");
}


int ElasIntfImpl::getDisp12MaxDiff() const
{
    throw std::runtime_error("not implemented");
}


void ElasIntfImpl::setDisp12MaxDiff(int disp12MaxDiff)
{
    throw std::runtime_error("not implemented");
}


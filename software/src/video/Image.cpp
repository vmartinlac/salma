#include "Image.h"

Image::Image()
{
    mValid = false;
}

void Image::setInvalid()
{
    mValid = false;
}

void Image::setValid(double timestamp, const cv::Mat& frame)
{
    mValid = true;
    mTimestamp = timestamp;
    mNumberOfFrames = 1;
    mFrames.at(0) = frame;
}

void Image::setValid(double timestamp, const cv::Mat& left_frame, const cv::Mat& right_frame)
{
    mValid = true;
    mTimestamp = timestamp;
    mNumberOfFrames = 2;
    mFrames.at(0) = left_frame;
    mFrames.at(1) = right_frame;
}

void Image::setValid(double timestamp, const std::vector<cv::Mat>& frames)
{
    if( frames.size() > mFrames.size() ) throw std::runtime_error("Too many frames!");
    mValid = true;
    mTimestamp = timestamp;
    mNumberOfFrames = frames.size();
    std::copy(frames.begin(), frames.end(), mFrames.begin());
}

bool Image::isValid()
{
    return mValid;
}

int Image::getNumberOfFrames()
{
    if( mValid )
    {
        return mNumberOfFrames;
    }
    else
    {
        return 0;
    }
}

double Image::getTimestamp()
{
    if( mValid )
    {
        return mTimestamp;
    }
    else
    {
        return 0.0;
    }
}

cv::Mat& Image::getFrame(int idx)
{
    if( mValid == false || idx < 0 || idx >= mNumberOfFrames )
    {
        throw std::runtime_error("Invalid image or incorrect frame number");
    }
    else
    {
        return mFrames.at(idx);
    }
}

void Image::merge( std::vector<Image>& from, Image& to )
{
    double timestamp = 0.0;
    std::vector<cv::Mat> frames;
    bool ok = true;

    if(ok)
    {
        ok = ( from.empty() == false );
    }

    if(ok)
    {
        for(Image& img : from)
        {
            ok = ok && img.isValid();
        }
    }

    if(ok)
    {
        timestamp = from.front().getTimestamp();

        for(Image& img : from)
        {
            frames.push_back( img.getFrame() );
        }

        to.setValid(timestamp, frames);
    }
    else
    {
        to.setInvalid();
    }
}

void Image::concatenate(Image& to)
{
    bool ok = true;

    if(ok)
    {
        ok = (mValid && mNumberOfFrames > 0);
    }

    if(ok)
    {
        for(int i=1; ok && i<mNumberOfFrames; i++)
        {
            ok = ok && (mFrames.front().type() == mFrames[i].type());
        }
    }

    if(ok)
    {
        int width = 0;
        int height = 0;

        for(int i=0; i<mNumberOfFrames; i++)
        {
            width += mFrames[i].cols;
            height = std::max(height, mFrames[i].rows);
        }

        cv::Mat output( cv::Size(width, height), mFrames.front().type() );
        output.setTo( cv::Scalar(0,0,0) );

        int delta = 0;
        for(int i=0; i<mNumberOfFrames; i++)
        {
            cv::Rect roi(delta, 0, mFrames[i].cols, mFrames[i].rows);
            mFrames[i].copyTo( output(roi) );
            delta += mFrames[i].cols;
        }

        to.setValid( mTimestamp, output);
    }
    else
    {
        to.setInvalid();
    }
}


#include <opencv2/imgcodecs.hpp>
#include "VideoReader.h"

VideoReader::VideoReader()
{
    mNumViews = 0;
}

VideoReader::~VideoReader()
{
}

void VideoReader::setFileName(const std::string& filename)
{
    mFileName = filename;
}

std::string VideoReader::getHumanName()
{
    std::string ret;
    ret = "{ " + mFileName + " }";
    return ret;
}

bool VideoReader::open()
{
    mCSVFile.open(mFileName);
    if(mCSVFile.is_open())
    {
        mCSVFile >> mNumViews;
        if(mNumViews != 1 && mNumViews != 2)
        {
            return false;
        }
        else
        {
            return true;
        }
    }
    else
    {
        return false;
    }
}

void VideoReader::close()
{
    mCSVFile.close();
}

void VideoReader::trigger()
{
}

void VideoReader::read(Image& image)
{
    // TODO
    image.setInvalid();
}

int VideoReader::getNumberOfCameras()
{
    return mNumViews;
}


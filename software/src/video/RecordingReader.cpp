#include <thread>
#include <opencv2/imgcodecs.hpp>
#include "RecordingReader.h"

RecordingReader::RecordingReader(RecordingHeaderPtr header, bool asynchronous)
{
    mAsynchronousLoading = asynchronous;
    mHeader = std::move(header);
    mIsOpen = false;
}

RecordingReader::~RecordingReader()
{
}

std::string RecordingReader::getHumanName()
{
    return std::string("Recording '") + mHeader->name + "' (" + std::to_string(mHeader->id) + ")";
}

bool RecordingReader::open()
{
    mIsOpen = true;
    mNextFrame = 0;
    return true;
}

void RecordingReader::close()
{
    mIsOpen = false;
}

Image RecordingReader::loadImage(RecordingHeaderPtr header, int rank)
{
    Image ret;
    std::vector<cv::Mat> views(header->num_views);
    bool ok = true;

    for(int i=0; ok && i<header->num_views; i++)
    {
        cv::Mat v = cv::imread( header->directory.absoluteFilePath(header->views[rank*header->num_views+i].filename).toStdString() );
        ok = bool(v.data);

        if(ok)
        {
            views[i] = std::move(v);
        }
    }

    if(ok)
    {
        ret.setValid( header->frames[rank].timestamp, views );
    }
    else
    {
        ret.setInvalid();
    }

    return ret;
}

void RecordingReader::trigger()
{
    if(mAsynchronousLoading)
    {
        if(0 <= mNextFrame && mNextFrame < mHeader->num_frames)
        {
            mNextImage = std::async(std::launch::async, loadImage, mHeader, mNextFrame);
            mNextFrame++;
        }
    }
}

void RecordingReader::read(Image& image)
{
    image.setInvalid();

    if(mAsynchronousLoading)
    {
        if(mNextImage.valid())
        {
            image = std::move( mNextImage.get() );
        }
    }
    else
    {
        image = loadImage(mHeader, mNextFrame);
    }
}

int RecordingReader::getNumberOfCameras()
{
    return mHeader->num_views;
}

void RecordingReader::seek(int frame)
{
    mNextFrame = frame;
    mNextImage = std::future<Image>();
}

RecordingHeaderPtr RecordingReader::getHeader()
{
    return mHeader;
}


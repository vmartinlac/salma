#include <thread>
#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include "RecordingReader.h"

RecordingReader::RecordingReader(RecordingHeaderPtr header)
{
    mHeader = std::move(header);
    mNextFrameId = 0;
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
    mNextFrameId = 0;
    return true;
}

void RecordingReader::close()
{
}

void RecordingReader::trigger()
{
}

void RecordingReader::read(Image& image)
{
    image.setInvalid();

    if( 0 <= mNextFrameId && mNextFrameId < mHeader->num_frames() )
    {
        bool ok = true;
        std::vector<cv::Mat> frames;

        for(int i=0; ok && i < mHeader->num_views(); i++)
        {
            cv::Mat mat = cv::imread( mHeader->getImageFileName(mNextFrameId, i).toStdString() );

            if(mat.data == nullptr)
            {
                ok = false;
            }
            else
            {
                frames.push_back(mat);
            }
        }

        if(ok)
        {
            image.setValid(mHeader->timestamps[mNextFrameId], frames);
            mNextFrameId++;
        }
    }

/*
    if( mVideo.isOpened() )
    {
        cv::Mat received;
        int current_frame;
        std::vector<cv::Mat> frames;
        bool ok = true;

        if(ok)
        {
            current_frame = static_cast<int>(mVideo.get(cv::CAP_PROP_POS_FRAMES));
            ok = (0 <= current_frame && current_frame < mHeader->num_frames());
        }

        if(ok)
        {
            ok = mVideo.read(received);
        }

        if(ok)
        {
            ok = (received.size() == mHeader->size);
        }

        if(ok)
        {
            const cv::Rect container(cv::Point2i(0,0), mHeader->size);

            frames.resize(mHeader->num_views());
            for(int i=0; ok && i<mHeader->num_views(); i++)
            {
                ok = (mHeader->views[i] | container) == container;

                if(ok)
                {
                    frames[i] = received(mHeader->views[i]);
                }
            }
        }

        if(ok)
        {
            image.setValid(mHeader->timestamps[current_frame], frames);
        }
    }
    */
}

int RecordingReader::getNumberOfCameras()
{
    return mHeader->num_views();
}

void RecordingReader::seek(int frame)
{
    if( 0 <= frame && frame < mHeader->num_frames() )
    {
        mNextFrameId = frame;
    }
    else
    {
        throw std::runtime_error("internal error");
    }
}

RecordingHeaderPtr RecordingReader::getHeader()
{
    return mHeader;
}


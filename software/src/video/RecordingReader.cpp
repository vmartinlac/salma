#include <thread>
#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include "RecordingReader.h"

/*

RecordingReader::RecordingReader(RecordingHeaderPtr header, bool unused)
{
    mHeader = std::move(header);
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
    bool ok = true;

    if(ok)
    {
        ok = mVideo.open(mHeader->filename);
    }

    if(ok)
    {
        const int frame_count = static_cast<int>(mVideo.get(cv::CAP_PROP_FRAME_COUNT));
        ok = (mHeader->num_frames() == frame_count);
    }

    return ok;
}

void RecordingReader::close()
{
    mVideo.release();
}

void RecordingReader::trigger()
{
}

void RecordingReader::read(Image& image)
{
    image.setInvalid();

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
}

int RecordingReader::getNumberOfCameras()
{
    return mHeader->num_views();
}

void RecordingReader::seek(int frame)
{
    if(mVideo.isOpened() && 0 <= frame && frame < mHeader->num_frames())
    {
        mVideo.set(cv::CAP_PROP_POS_FRAMES, frame);
    }
}

RecordingHeaderPtr RecordingReader::getHeader()
{
    return mHeader;
}
*/


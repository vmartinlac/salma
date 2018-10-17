#include <stdexcept>
#include "AssembledVideoSource.h"

AssembledVideoSource::AssembledVideoSource()
{
}

AssembledVideoSource::~AssembledVideoSource()
{
    close();
}

std::string AssembledVideoSource::getHumanName()
{
    return "Composite video source";
}

bool AssembledVideoSource::open()
{
    bool ok = true;

    for(VideoSourcePtr& ptr : mVideoSources)
    {
        ok = ok && ptr->open();
    }

    if(ok)
    {
        return true;
    }
    else
    {
        close();
        return false;
    }
}

void AssembledVideoSource::close()
{
    for(VideoSourcePtr& ptr : mVideoSources)
    {
        ptr->close();
    }
}

void AssembledVideoSource::trigger()
{
    for(VideoSourcePtr& ptr : mVideoSources)
    {
        ptr->trigger();
    }
}

void AssembledVideoSource::read(Image& image)
{
    double timestamp = 0.0;

    std::vector<cv::Mat> frames;
    frames.resize(getNumberOfCameras());

    bool ok = true;
    int j = 0;

    for(int i=0; ok && i<mVideoSources.size(); i++)
    {
        Image tmp;
        mVideoSources[i]->read(tmp);

        if( tmp.isValid() )
        {
            if(i == 0 )
            {
                timestamp = tmp.getTimestamp();
            }

            for(int k=0; k<tmp.getNumberOfFrames(); k++)
            {
                frames.at(j) = tmp.getFrame(k);
                j++;
            }
        }
        else
        {
            ok = false;
        }
    }

    if(ok)
    {
        if( j != frames.size() )
        {
            throw std::runtime_error("Internal error");
        }

        image.setValid(timestamp, frames);
    }
    else
    {
        image.setInvalid();
    }
}

void AssembledVideoSource::setVideoSources(const std::vector<VideoSourcePtr>& video_sources)
{
    mVideoSources = video_sources;
}

int AssembledVideoSource::getNumberOfCameras()
{
    int sum = 0;

    for(VideoSourcePtr& ptr : mVideoSources)
    {
        sum += ptr->getNumberOfCameras();
    }

    return sum;
}

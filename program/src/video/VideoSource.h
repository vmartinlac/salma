#pragma once

#include <memory>
#include <string>
#include "Image.h"
#include "VideoErrorCode.h"

class VideoSource
{
public:

    VideoSource();
    virtual ~VideoSource();

    virtual std::string getHumanName() = 0;

    virtual bool open() = 0;
    virtual void close() = 0;

    virtual void trigger() = 0;
    virtual void read(Image& image) = 0;

    virtual int getNumberOfCameras() = 0;
};

typedef std::shared_ptr<VideoSource> VideoSourcePtr;


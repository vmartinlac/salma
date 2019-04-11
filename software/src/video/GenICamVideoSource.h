#pragma once

#include "VideoSource.h"

class GenICamVideoSource : public VideoSource
{
public:

    virtual void setSoftwareTrigger() = 0;
    virtual void setHardwareTrigger(const std::string& device) = 0;
};

typedef std::shared_ptr<GenICamVideoSource> GenICamVideoSourcePtr;


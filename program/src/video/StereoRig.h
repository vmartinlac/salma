#pragma once

#include <memory>
#include <string>
#include "StereoImage.h"

class StereoRig;

typedef std::shared_ptr<StereoRig> StereoRigPtr;

class StereoRig
{
public:

    static StereoRigPtr createFromRecording(const std::string& filename);

    virtual bool open() = 0;
    virtual void close() = 0;

    virtual bool trigger() = 0;
    virtual bool read(StereoImage& to) = 0;
};

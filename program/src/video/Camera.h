#pragma once

#include <memory>
#include <string>
#include "Image.h"

class Camera
{
public:

    Camera();
    virtual ~Camera();

    virtual bool open() = 0;
    virtual void close() = 0;

    virtual std::string getHumanName() = 0;
    virtual void read(Image& image) = 0;
};

typedef std::shared_ptr<Camera> CameraPtr;


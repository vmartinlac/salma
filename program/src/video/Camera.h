#pragma once

#include <memory>
#include <string>
#include "Image.h"
#include "Trigger.h"

class Camera : public Trigger
{
public:

    Camera();
    virtual ~Camera();

    virtual std::string getHumanName() = 0;
    virtual void read(Image& image) = 0;
};

typedef std::shared_ptr<Camera> CameraPtr;


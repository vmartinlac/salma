#pragma once

#include <memory>
#include "Camera.h"
#include "Image.h"

class VimbaCameraManager
{

public:

    static VimbaCameraManager& instance();

    virtual bool initialize() = 0;

    virtual void finalize() = 0;

    virtual std::shared_ptr<Camera> getDefaultCamera() = 0;

    virtual int getNumCameras() = 0;

    virtual std::shared_ptr<Camera> getCamera(int id) = 0;
};


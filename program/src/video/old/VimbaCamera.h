#pragma once

#include <memory>
#include "Camera.h"
#include "Image.h"
#include "StereoRig.h"

class VimbaCameraManager
{

public:

    static VimbaCameraManager& instance();

    virtual bool initialize() = 0;

    virtual void finalize() = 0;

    virtual std::shared_ptr<Camera> getDefaultCamera() = 0;

    virtual int getNumCameras() = 0;

    virtual std::shared_ptr<Camera> getCamera(int id) = 0;

    virtual StereoRigPtr createStereoRig(int left_camera, int right_camera) = 0;
};


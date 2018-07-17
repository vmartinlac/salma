#pragma once

#include <string>

class Camera
{

public:

    virtual ~Camera();

    virtual std::string humanName() = 0;
    virtual void start() = 0;
    virtual void stop() = 0;
    virtual void readNextImage() = 0;
};

class CameraManager
{

public:

    virtual ~CameraManager();

    static CameraManager* createDefaultCameraManager();

    virtual bool initialize() = 0;
    virtual void finalize() = 0;

    virtual int getNumCameras() = 0;
    virtual Camera* getCamera(int id) = 0;
    virtual Camera* getDefaultCamera() = 0;
};


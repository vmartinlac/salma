#pragma once

#include <string>

class Image;

class Camera
{

public:

    Camera(int id);
    virtual ~Camera();

    int getId();

    virtual std::string getHumanName() = 0;

    virtual bool open() = 0;
    virtual void close() = 0;

    /*
    This function returns the most recent frame received from the camera since the last call.
    If no frame was received since last call, this function blocks.
    This function is intended to be called after a call to start() and before a call to stop().
    */
    virtual bool read(Image& image) = 0;

private:

    int m_id;
};

class CameraManager
{

public:

    virtual ~CameraManager();

    static CameraManager* createVimbaCameraManager();
    static CameraManager* createPseudoCameraManager();
    static CameraManager* createOpenCVCameraManager();

    virtual bool initialize() = 0;
    virtual void finalize() = 0;

    virtual int getNumCameras() = 0;
    virtual Camera* getCamera(int id) = 0;
    virtual Camera* getDefaultCamera() = 0;
};


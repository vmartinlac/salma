#pragma once

#include <string>

class Camera
{

public:

    Camera(int id);
    virtual ~Camera();

    int getId();
    virtual std::string getHumanName() = 0;
    virtual bool start() = 0;
    virtual void stop() = 0;
    //virtual void readNextImage() = 0;

private:

    int m_id;
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


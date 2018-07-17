#include <iostream>
#include <vector>
#include <VimbaC/Include/VimbaC.h>
#include "Camera.h"

Camera::~Camera()
{
    ;
}

CameraManager::~CameraManager()
{
    ;
}

class VimbaCamera : public Camera
{
public:
protected:
};

class VimbaCameraManager : public CameraManager
{
public:
    bool initialize() override
    {
        VmbStartup();

        VmbError_t err;
        VmbUint32_t count;
        VmbCameraInfo_t* info;

        err = VmbCamerasList(NULL, 0, &count, sizeof(*info));

        if(err != VmbErrorSuccess) throw;
        std::cout << count << std::endl;
    }

    void finalize() override
    {
        VmbShutdown();
    }

    Camera* getDefaultCamera() override
    {
        return nullptr;
    }

    int getNumCameras() override
    {
        return 0;
    }

    Camera* getCamera(int id) override
    {
        return nullptr;
    }

protected:
};

CameraManager* CameraManager::createDefaultCameraManager()
{
    return new VimbaCameraManager();
}

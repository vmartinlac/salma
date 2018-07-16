#pragma once

#include <mutex>
#include <memory>

class CameraPrivate;

class Camera
{
public:

    static Camera* create();

    Camera(Camera&& o) = delete;
    Camera(const Camera& o) = delete;
    void operator=(const Camera& o) = delete;

    void initialize();
    void start();
    void stop();
    void readNextImage();

protected:
    Camera();
    CameraPrivate* d;
};


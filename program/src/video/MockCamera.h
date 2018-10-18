#pragma once

#include <memory>
#include <chrono>
#include "Camera.h"

class MockCamera : public Camera
{
public:

    MockCamera(int width, int height);
    ~MockCamera() override;

    bool open() override;
    void close() override;

    std::string getHumanName() override;

    void read(Image& image) override;
    void trigger() override;

protected:

    int mWidth;
    int mHeight;
    bool mReady;
    std::chrono::time_point<std::chrono::steady_clock> mT0;
};

typedef std::shared_ptr<MockCamera> MockCameraPtr;


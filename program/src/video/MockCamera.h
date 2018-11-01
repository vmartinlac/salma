#pragma once

#include <memory>
#include <chrono>
#include "VideoSource.h"

class MockCamera : public VideoSource
{
public:

    MockCamera(int num_views, int width, int height);
    ~MockCamera() override;

    bool open() override;
    void close() override;

    std::string getHumanName() override;
    int getNumberOfCameras() override;

    void read(Image& image) override;
    void trigger() override;

protected:

    int mWidth;
    int mHeight;
    int mNumViews;
    bool mReady;
    std::chrono::time_point<std::chrono::steady_clock> mT0;
};

typedef std::shared_ptr<MockCamera> MockCameraPtr;


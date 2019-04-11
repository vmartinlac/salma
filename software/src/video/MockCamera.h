#pragma once

#include <memory>
#include <chrono>
#include "GenICamVideoSource.h"

class MockCamera : public GenICamVideoSource
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

    void setSoftwareTrigger() override;
    void setHardwareTrigger(const std::string& device) override;

protected:

    int mWidth;
    int mHeight;
    int mNumViews;
    bool mReady;
    std::chrono::time_point<std::chrono::steady_clock> mT0;
};

typedef std::shared_ptr<MockCamera> MockCameraPtr;


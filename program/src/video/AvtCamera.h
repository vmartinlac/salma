#pragma once

#include <memory>
#include <condition_variable>
#include <mutex>
#include <VimbaCPP/Include/VimbaCPP.h>
#include "VideoSource.h"

class AvtCamera : public VideoSource
{
public:

    AvtCamera(AVT::VmbAPI::CameraPtr camera);
    ~AvtCamera() override;

    bool open() override;
    void close() override;

    std::string getHumanName() override;

    void read(Image& image) override;
    void trigger() override;

    int getNumberOfCameras() override;

protected:

    class FrameObserver;

protected:

    bool mIsOpen;
    VmbInt64_t mTickFrequency;
    AVT::VmbAPI::FramePtrVector mFrames;
    AVT::VmbAPI::CameraPtr mCamera;

    std::mutex mMutex;
    std::condition_variable mCondition;
    Image mNewImage;
};

typedef std::shared_ptr<AvtCamera> AvtCameraPtr;


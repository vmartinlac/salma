#pragma once

#include <memory>
#include <mutex>
#include <VimbaCPP/Include/VimbaCPP.h>
#include "Camera.h"

class AvtCamera : public Camera
{
public:

    AvtCamera(AVT::VmbAPI::CameraPtr camera);
    ~AvtCamera() override;

    bool open() override;
    void close() override;

    std::string getHumanName() override;

    void read(Image& image) override;
    void trigger() override;

protected:

    class FrameObserver;

protected:

    bool mIsOpen;
    VmbInt64_t mTickFrequency;
    AVT::VmbAPI::FramePtrVector mFrames;
    AVT::VmbAPI::CameraPtr mCamera;

    std::mutex mMutex;
    Image mNewImage;
};

typedef std::shared_ptr<AvtCamera> AvtCameraPtr;


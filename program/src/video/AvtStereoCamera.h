#pragma once

#include <future>
#include <memory>
#include <string>
#include <VimbaCPP/Include/VimbaCPP.h>
#include "VideoSource.h"
#include "Image.h"

class AvtStereoCamera : public VideoSource
{
public:

    AvtStereoCamera();
    ~AvtStereoCamera() override;

    void setCamera(AVT::VmbAPI::CameraPtr camera);
    void setCameras(AVT::VmbAPI::CameraPtr left_camera, AVT::VmbAPI::CameraPtr right_camera);

    std::string getHumanName() override;

    bool open() override;
    void close() override;

    void trigger() override;
    void read(Image& image) override;

    int getNumberOfCameras() override;

protected:

    struct Camera
    {
        AVT::VmbAPI::CameraPtr camera;
        AVT::VmbAPI::FramePtr frame;
        std::promise<Image> image;

        VmbInt64_t tick_frequency;
        VmbInt64_t payload_size;
    };

    class FrameObserver;

    std::vector<Camera> mCameras;

protected:

    void setImage(int idx, Image& image);
};

typedef std::shared_ptr<VideoSource> VideoSourcePtr;


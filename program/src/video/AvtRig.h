#pragma once

#include <condition_variable>
#include <mutex>
#include <memory>
#include <string>
#include <VimbaCPP/Include/VimbaCPP.h>
#include "VideoSource.h"
#include "Image.h"

class AvtRig : public VideoSource
{
public:

    AvtRig();
    AvtRig(std::initializer_list<AVT::VmbAPI::CameraPtr> cameras);

    void setCameras(std::initializer_list<AVT::VmbAPI::CameraPtr> cameras);
    //void setMode(); // TRIGGERED, FREERUN

    ~AvtRig() override;

    std::string getHumanName() override;

    bool open() override;
    void close() override;

    void trigger() override;
    void read(Image& image) override;

    int getNumberOfCameras() override;

protected:

    class CameraData;

    typedef std::shared_ptr<CameraData> CameraDataPtr;

    class FrameObserver;

protected:

    std::vector<CameraDataPtr> mCameras;
    std::mutex mMutex;
    std::condition_variable mCondition;
};

typedef std::shared_ptr<AvtRig> AvtRigPtr;


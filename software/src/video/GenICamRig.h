#pragma once

#include <iostream>
#include <atomic>
#include <map>
#include <chrono>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <memory>
#include <initializer_list>
#include <vector>
#include <opencv2/core.hpp>

#include "VideoSource.h"
#include "GenICamCamera.h"
#include "GenICamSemaphore.h"

class GenICamRig : public VideoSource
{
public:

    GenICamRig(
        const std::initializer_list<std::string>& cameras,
        bool software_trigger);

    virtual ~GenICamRig();

    void setCameras(const std::initializer_list<std::string>& cameras);

    bool open() override;

    void close() override;

    void read(Image& image) override;

    void trigger();

    std::string getHumanName() override;

    int getNumberOfCameras() override;

    void signalImageAvailability();

protected:

    void produceImages();

protected:

    class Counter;

protected:

    bool mIsOpen;
    bool mSoftwareTrigger;
    std::vector<GenICamCameraPtr> mCameras;
    std::thread mThread;
    GenICamSemaphore mSemaphore;
    std::atomic<bool> mAskThreadToQuit;

    Image mImage;
    std::timed_mutex mMutexA;
    std::mutex mMutexB;
};

typedef std::shared_ptr<GenICamRig> GenICamRigPtr;


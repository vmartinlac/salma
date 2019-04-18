#pragma once

#include <iostream>
#include <atomic>
#include <map>
#include <chrono>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <memory>
#include <vector>
#include <opencv2/core.hpp>
#include "ExternalTrigger.h"
#include "GenICamVideoSource.h"
#include "GenICamCamera.h"
#include "GenICamSemaphore.h"

class GenICamRig : public GenICamVideoSource
{
public:

    GenICamRig( const std::vector<std::string>& cameras );

    virtual ~GenICamRig();

    void setCameras(const std::vector<std::string>& cameras);

    bool open() override;

    void close() override;

    void read(Image& image) override;

    void trigger() override;

    std::string getHumanName() override;

    int getNumberOfCameras() override;

    void signalImageAvailability();

    void setSoftwareTrigger() override;

    void setHardwareTrigger(const std::string& device) override;

protected:

    void produceImages();

    static cv::Mat convertBufferToMap(ArvBuffer* buffer);

protected:

    bool mIsOpen;
    ExternalTriggerPtr mTrigger;
    std::vector<GenICamCameraPtr> mCameras;
    std::thread mThread;
    GenICamSemaphore mSemaphore;
    std::atomic<bool> mAskThreadToQuit;

    Image mImage;
    std::timed_mutex mMutexA;
    std::mutex mMutexB;

    bool mHasFirstTimestamp;
    double mFirstTimestamp;
};

typedef std::shared_ptr<GenICamRig> GenICamRigPtr;


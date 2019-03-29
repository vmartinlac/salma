
#pragma once

#include <iostream>
#include <map>
#include <chrono>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <memory>
#include <vector>
#include <arv.h>
#include <opencv2/core.hpp>

#include "Camera.h"

class Semaphore
{
public:

    Semaphore(int count = 0) : mCount(count)
    {
    }

    void up()
    {
        std::unique_lock<std::mutex> lock(mMutex);
        mCount++;
        mCondition.notify_one();
    }

    void down()
    {
        std::unique_lock<std::mutex> lock(mMutex);

        while(mCount == 0)
        {
            mCondition.wait(lock);
        }

        mCount--;
    }

private:

    std::mutex mMutex;
    std::condition_variable mCondition;
    int mCount;
};

class Image
{
public:

    Image()
    {
        mValid = false;
    }

    Image(std::vector<cv::Mat>&& vec)
    {
        mViews = vec;
    }

    Image(Image&& o)
    {
        mViews = std::move(o.mViews);
    }

    void operator=(Image&& o)
    {
        mViews = std::move(o.mViews);
    }

    void setInvalid()
    {
        mValid = false;
        mViews.clear();
    }

    void setValid(
        double timestamp,
        int frameid,
        std::vector<cv::Mat>& frames)
    {
    }

protected:

    bool mValid;
    double mCameraTimestamp;
    double mSystemTimestamp;
    int mCameraFrameId;
    int mSystemFrameId;
    std::vector<cv::Mat> mViews;
};

class Rig
{
public:

    Rig(const std::initializer_list<int>& cams);

    void open();

    void close();

    bool read(Image& im);

    void trigger();

protected:

    static void RigProc(Rig* rig);

public:

    bool mIsOpen;
    std::vector<CameraPtr> mCameras;
    std::thread mThread;
    Semaphore mSemaphore;
    bool mAskThreadToQuit;

    Image mImage;
    std::timed_mutex mMutexA;
    std::mutex mMutexB;
};

typedef std::shared_ptr<Rig> RigPtr;


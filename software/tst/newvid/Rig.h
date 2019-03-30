
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

    void operator=(Image&& o)
    {
        mValid = o.mValid;

        if(mValid)
        {
            mFrameId = o.mFrameId;
            mTimestamp = o.mTimestamp;
            mViews = std::move(o.mViews);
        }
        else
        {
            mViews.clear();
        }

        o.setInvalid();
    }

    void setInvalid()
    {
        mValid = false;
        mViews.clear();
    }

    void setValid(
        int frameid,
        double timestamp,
        std::vector<cv::Mat>& frames)
    {
        mValid = true;
        mFrameId = frameid;
        mTimestamp = timestamp;
        mViews = frames;
    }

    bool isValid()
    {
        return mValid;
    }

    int getFrameId()
    {
        return mFrameId;
    }

protected:

    bool mValid;
    double mTimestamp;
    int mFrameId;
    std::vector<cv::Mat> mViews;
};

class Rig
{
public:

    Rig(const std::initializer_list<int>& cams);

    void open();

    void close();

    void read(Image& im);

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


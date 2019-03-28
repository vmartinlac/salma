
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

class Rig
{
public:

    Rig(const std::initializer_list<int>& cams);

    void open();

    void close();

    cv::Mat read();

protected:

    static void RigProc(Rig* rig);

public:

    bool mIsOpen;
    std::vector<CameraPtr> mCameras;
    std::thread mThread;
    Semaphore mSemaphore;
    bool mAskThreadToQuit;
};

typedef std::shared_ptr<Rig> RigPtr;



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

    std::vector<CameraPtr> mCameras;
    //std::mutex mMutex;
    std::thread mThread;
    std::condition_variable mConditionVariable;
};

typedef std::shared_ptr<Rig> RigPtr;


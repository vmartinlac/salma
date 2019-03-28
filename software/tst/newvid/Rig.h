
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
#include <QSemaphore>

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

    bool mIsOpen;
    std::vector<CameraPtr> mCameras;
    std::thread mThread;
    QSemaphore mSemaphore;
};

typedef std::shared_ptr<Rig> RigPtr;


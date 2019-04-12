
#pragma once

#include <iostream>
#include <map>
#include <chrono>
#include <thread>
#include <mutex>
#include <memory>
#include <vector>
#include <arv.h>
#include <opencv2/core.hpp>

class GenICamRig;

class GenICamCamera
{
public:

    GenICamCamera(
        GenICamRig* rig,
        const std::string& id,
        int rank);

    bool open(bool software_trigger);

    void close();

    void softwareTrigger();

    std::string getId();

protected:

    static void stream_callback(ArvStream *stream, void *user_data);

public:

    GenICamRig* mRig;
    int mRank;
    std::string mId;

    ArvCamera* mCamera;
    ArvDevice* mDevice;
    ArvStream* mStream;

    std::mutex mReceivedBufferMutex;
    ArvBuffer* mReceivedBuffer;
};

typedef std::shared_ptr<GenICamCamera> GenICamCameraPtr;


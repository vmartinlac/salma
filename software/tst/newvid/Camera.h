
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

class Rig;

class Camera
{
public:

    Camera(Rig* rig, const std::string& id, int rank);

    void open();

    void close();

protected:

    static void stream_callback(void* user_data, ArvStreamCallbackType type, ArvBuffer* buffer);

public:

    Rig* mRig;
    int mRank;
    std::string mId;

    ArvStream* mStream;
    ArvDevice* mDevice;

    std::mutex mMutex;
    std::map<guint32,ArvBuffer*> mBuffers;
};

typedef std::shared_ptr<Camera> CameraPtr;

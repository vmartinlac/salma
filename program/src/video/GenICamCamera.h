#pragma once

#include <chrono>
#include <memory>
#include <arv.h>

class GenICamRig;

class GenICamCamera
{
public:

    GenICamCamera(GenICamRig* rig, const std::string& id);
    ~GenICamCamera();

    std::string getId();

    bool open();
    void close();

    void trigger();
    void read(std::chrono::time_point<std::chrono::steady_clock> tmax, Image& im);

protected:

    bool mIsOpen;
    ArvDevice* mDevice;
    ArvStream* mStream;
    ArvBuffer* mBuffer;
    gint64 mPayload;
    GenICamRig* mRig;
    std::string mId;
    bool mFirstFrame;
    guint64 mFirstTimestamp;
};

typedef std::shared_ptr<GenICamCamera> GenICamCameraPtr;

extern "C" void GenICamCallback(void* user_data, ArvStreamCallbackType type, ArvBuffer* buffer);


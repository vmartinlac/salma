#pragma once

#include <mutex>
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

    void onFrameReceived();
    void takeLastImage(Image& image);

protected:

    std::mutex mMutex;
    bool mIsOpen;
    ArvDevice* mDevice;
    ArvStream* mStream;
    std::vector<ArvBuffer*> mAvailableBuffers;
    gint64 mPayload;
    GenICamRig* mRig;
    std::string mId;
    bool mFirstFrame;
    guint64 mFirstTimestamp;
    Image mLastImage;
};

typedef std::shared_ptr<GenICamCamera> GenICamCameraPtr;

//extern "C" void GenICamCallback(void* user_data, ArvStreamCallbackType type, ArvBuffer* buffer);
extern "C" void GenICamCallback(ArvStream* stream, void* data);

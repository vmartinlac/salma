
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

#include "GenICamConfig.h"

class Rig;

template<typename T, int D>
class CircularBuffer
{
public:

    CircularBuffer()
    {
        mFirst = 0;
        mCount = 0;
    }

    constexpr int maxsize()
    {
        return D;
    }

    template<typename Iterator>
    void take(Iterator it)
    {
        mMutex.lock();

        for(int i=0; i<mCount; i++)
        {
            *it = mTab[ (mFirst + i) % D ];
            it++;
        }

        mCount = 0;
        mFirst = 0;

        mMutex.unlock();
    }

    T* push(T* item)
    {
        T* ret = nullptr;

        mMutex.lock();

        if(mCount < D)
        {
            mTab[mCount] = item;
            mCount++;
        }
        else
        {
            T*& cell = mTab[ (mFirst + D - 1) % D ];
            ret = cell;
            cell = item;
            mFirst = (mFirst+1) % D;
        }

        mMutex.unlock();

        return ret;
    }

    void clear()
    {
        mMutex.lock();
        mFirst = 0;
        mCount = 0;
        mMutex.unlock();
    }

protected:

    T* mTab[D];
    int mCount;
    int mFirst;
    std::mutex mMutex;
};

class Camera
{
public:

    Camera(Rig* rig, const std::string& id, int rank);

    void open();

    void close();

    void trigger();

protected:

    static void stream_callback(void* user_data, ArvStreamCallbackType type, ArvBuffer* buffer);

public:

    Rig* mRig;
    int mRank;
    std::string mId;

    ArvDevice* mDevice;
    ArvStream* mStream;

    CircularBuffer<ArvBuffer, GENICAM_NUM_BUFFERS> mTab1;
    std::map<guint32,ArvBuffer*> mTab2;
};

typedef std::shared_ptr<Camera> CameraPtr;



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

public:

    GenICamRig* mRig;
    int mRank;
    std::string mId;

    ArvCamera* mCamera;
    ArvDevice* mDevice;
    ArvStream* mStream;

    CircularBuffer<ArvBuffer, GENICAM_NUM_BUFFERS> mTab1;
    std::map<guint32,ArvBuffer*> mTab2;
};

typedef std::shared_ptr<GenICamCamera> GenICamCameraPtr;


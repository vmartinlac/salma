#include <iostream>
#include <set>
#include <thread>
#include "GenICamRig.h"
#include "ArduinoTrigger.h"

GenICamRig::GenICamRig(const std::vector<std::string>& cameras)
{
    mHasFirstTimestamp = false;
    mFirstTimestamp = 0.0;
    mIsOpen = false;
    setCameras(cameras);
}

void GenICamRig::setCameras(const std::vector<std::string>& cameras)
{
    // check that the rig is not currently open.

    if(mIsOpen) throw std::runtime_error("can not change list of cameras while rig is open");

    // check that camera are different.

    {
        std::set<std::string> set(cameras.begin(), cameras.end());
        if( set.size() != cameras.size() ) throw std::runtime_error("A camera can appear at most once in a rig.");
    }

    // apply changes.

    mCameras.clear();

    int rank = 0;
    for(const std::string& id : cameras)
    {
        GenICamCameraPtr cam( new GenICamCamera(this, id, rank) );
        mCameras.push_back(cam);
        rank++;
    }
}

GenICamRig::~GenICamRig()
{
    if(mIsOpen)
    {
        close();
    }
}


std::string GenICamRig::getHumanName()
{
    std::string ret;
    bool first = true;

    ret = "{ ";

    for(GenICamCameraPtr c : mCameras)
    {
        if(first)
        {
            first = false;
        }
        else
        {
            ret += " ; ";
        }

        ret += c->getId();
    }

    ret += " }";

    return ret;
}

bool GenICamRig::open()
{
    bool ok = true;

    if(mIsOpen) throw std::runtime_error("camera is already open");

    mHasFirstTimestamp = false;
    mFirstTimestamp = 0.0;

    const bool software_trigger = !bool(mTrigger);

    if(ok && bool(mTrigger))
    {
        ok = mTrigger->open();
    }

    for(size_t i = 0; ok && i<mCameras.size(); i++)
    {
        ok = mCameras[i]->open(software_trigger);
    }

    if(ok)
    {
        mAskThreadToQuit = false;

        mThread = std::thread([this] () { produceImages(); });

        const bool locked = mMutexA.try_lock();
        if(locked == false) throw std::runtime_error("internal error");
    }

    mIsOpen = ok;

    return ok;
}

void GenICamRig::close()
{
    if(mIsOpen)
    {

        mAskThreadToQuit = true;
        mSemaphore.up();
        mThread.join();

        for( GenICamCameraPtr cam : mCameras )
        {
            cam->close();
        }

        if(mTrigger)
        {
            mTrigger->close();
        }

        mImage.setInvalid();
        mIsOpen = false;
    }
}

void GenICamRig::trigger()
{
    for(GenICamCameraPtr cam : mCameras)
    {
        cam->mReceivedBufferMutex.lock();
        if(cam->mReceivedBuffer)
        {
            arv_stream_push_buffer( cam->mStream, cam->mReceivedBuffer );
            cam->mReceivedBuffer = nullptr;
        }
        cam->mReceivedBufferMutex.unlock();
    }

    if(mTrigger)
    {
        mTrigger->trigger();
    }
    else
    {
        for(GenICamCameraPtr c : mCameras)
        {
          c->softwareTrigger();
        }
    }
}

void GenICamRig::read(Image& im)
{
    im.setInvalid();

    const bool locked = mMutexA.try_lock_for(std::chrono::milliseconds(200));

    if(locked)
    {
        mMutexB.lock();
        im = std::move(mImage);
        mMutexB.unlock();
    }
}

int GenICamRig::getNumberOfCameras()
{
    return static_cast<int>(mCameras.size());
}

void GenICamRig::produceImages()
{
    bool go_on = true;

    const int N_views = static_cast<int>(mCameras.size());

    double timestamp = 0.0;
    std::vector<cv::Mat> planes(N_views);

    while(go_on)
    {
        mSemaphore.down();

        if(mAskThreadToQuit)
        {
            go_on = false;
        }
        else
        {
            bool ready = true;

            for(size_t i=0; i<N_views; i++)
            {
                ArvBuffer* buffer = nullptr;

                mCameras[i]->mReceivedBufferMutex.lock();
                buffer = mCameras[i]->mReceivedBuffer;
                mCameras[i]->mReceivedBuffer = nullptr;
                mCameras[i]->mReceivedBufferMutex.unlock();

                if(buffer)
                {
                    planes[i] = convertBufferToMap(buffer);

                    if(i == 0)
                    {
                        const double this_timestamp = static_cast<double>(arv_buffer_get_timestamp(buffer)) * 1.0e-9;

                        if(mHasFirstTimestamp == false)
                        {
                            mHasFirstTimestamp = true;
                            mFirstTimestamp = this_timestamp;
                        }

                        timestamp = this_timestamp - mFirstTimestamp;
                    }

                    arv_stream_push_buffer(mCameras[i]->mStream, buffer);
                }

                ready = ready && bool(planes[i].data);
            }

            if(ready)
            {
                Image image;
                image.setValid(timestamp, planes);

                timestamp = 0.0;
                planes.assign( N_views, cv::Mat() );

                mMutexB.lock();
                mImage = std::move(image);
                mMutexB.unlock();

                mMutexA.unlock();
            }
        }
    }
}

void GenICamRig::signalImageAvailability()
{
    mSemaphore.up();
}

void GenICamRig::setSoftwareTrigger()
{
    mTrigger.reset();
}

void GenICamRig::setHardwareTrigger(const std::string& device)
{
    ArduinoTriggerPtr trigger(new ArduinoTrigger());
    trigger->setPathToSerialPort(device);
    mTrigger = trigger;
}

cv::Mat GenICamRig::convertBufferToMap(ArvBuffer* buffer)
{
    const gint width = arv_buffer_get_image_width(buffer);
    const gint height = arv_buffer_get_image_height(buffer);

    size_t size;
    const uint8_t* data = reinterpret_cast<const uint8_t*>(arv_buffer_get_data(buffer, &size));

    if(size != width*height*3) throw std::runtime_error("internal error");

    cv::Mat mat( cv::Size(width, height), CV_8UC3 );
    std::copy( data, data+size, mat.data );

    return mat;
}


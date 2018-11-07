#include <iostream>
#include <set>
#include <thread>
#include "GenICamRig.h"

GenICamRig::GenICamRig(std::initializer_list<std::string> cameras)
{
    mIsOpen = false;
    setCameras(cameras);
}

GenICamRig::~GenICamRig()
{
    if(mIsOpen)
    {
        close();
    }
}

void GenICamRig::setCameras(std::initializer_list<std::string> cameras)
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

    for(const std::string& id : cameras)
    {
        mCameras.push_back( GenICamCameraPtr(new GenICamCamera(this, id)) );
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
    if(mIsOpen) throw std::runtime_error("camera is already open");

    mIsOpen = true;

    if(mExternalTrigger)
    {
        mIsOpen = mIsOpen && mExternalTrigger->open();
    }

    for(GenICamCameraPtr c : mCameras)
    {
        mIsOpen = mIsOpen && c->open();
    }

    return mIsOpen;
}

void GenICamRig::close()
{
    if(mIsOpen)
    {
        for(GenICamCameraPtr c : mCameras)
        {
            c->close();
        }

        if(mExternalTrigger)
        {
            mExternalTrigger->close();
        }

        mIsOpen = false;
    }
}

void GenICamRig::trigger()
{
    for(GenICamCameraPtr c : mCameras)
    {
        c->prepareTrigger();
    }

    if(mExternalTrigger)
    {
        mExternalTrigger->trigger();
    }
    else
    {
      for(GenICamCameraPtr c : mCameras)
      {
          c->softwareTrigger();
      }
    }
}

void GenICamRig::read(Image& image)
{
    std::unique_lock<std::mutex> lock(mMutex);

    //auto expiration_time = std::chrono::steady_clock::now() + std::chrono::milliseconds(150);
    const size_t N = mCameras.size();

    std::vector<Image> images(N);

    auto pred = [&images,this,N] ()
    {
        bool ret = true;

        for(size_t i=0; i<N; i++)
        {
            if(images[i].isValid() == false)
            {
                mCameras[i]->takeLastImage(images[i]);
            }

            ret = ret && images[i].isValid();
        }

        return ret;
    };

    const bool ok = mCondition.wait_for(lock, std::chrono::milliseconds(100), pred);

    if(ok)
    {
        Image::merge(images, image);
    }
    else
    {
        image.setInvalid();
    }
}

int GenICamRig::getNumberOfCameras()
{
    return mCameras.size();
}

void GenICamRig::onFrameReceived()
{
    std::lock_guard<std::mutex> lock(mMutex);
    mCondition.notify_one();
}

void GenICamRig::setExternalTrigger(ExternalTriggerPtr trigger)
{
  mExternalTrigger = std::move(trigger);
}


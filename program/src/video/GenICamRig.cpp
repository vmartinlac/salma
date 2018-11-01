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

        mIsOpen = false;
    }
}

void GenICamRig::trigger()
{
    for(GenICamCameraPtr c : mCameras)
    {
        c->trigger();
    }
}

void GenICamRig::read(Image& image)
{
    auto expiration_time = std::chrono::steady_clock::now() + std::chrono::milliseconds(150);

    const size_t N = mCameras.size();

    std::vector<Image> images(N);

    for(size_t i=0; i<N; i++)
    {
        GenICamCameraPtr c = mCameras[i];

        c->read(expiration_time, images[i]);
    }

    Image::merge(images, image);
}

int GenICamRig::getNumberOfCameras()
{
    return mCameras.size();
}


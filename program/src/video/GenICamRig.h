#pragma once

#include <initializer_list>
#include <memory>
#include "VideoSource.h"
#include "GenICamCamera.h"

class GenICamRig : public VideoSource
{
public:

    GenICamRig(std::initializer_list<std::string> cameras);
    virtual ~GenICamRig();

    void setCameras(std::initializer_list<std::string> cameras);

    std::string getHumanName() override;

    bool open() override;
    void close() override;

    void trigger() override;
    void read(Image& image) override;

    int getNumberOfCameras() override;

protected:

    bool mIsOpen;
    std::vector<GenICamCameraPtr> mCameras;
};

typedef std::shared_ptr<GenICamRig> GenICamRigPtr;


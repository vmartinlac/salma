#include "Tracker.h"
#include "KFDFilter.h"
#include "KFDEngine.h"

KFDEngine::KFDEngine(QObject* parent) : QThread(parent)
{
    mPosePort = new KFDPosePort(this);
}


KFDEngine::~KFDEngine()
{
}

KFDPosePort* KFDEngine::posePort()
{
    return mPosePort;
}

void KFDEngine::run()
{
    target::Tracker tracker;
    KFDFilter filter;

    filter.init();

    mCamera->open();
    mCamera->trigger();

    while( isInterruptionRequested() == false )
    {
        Image image;

        mCamera->read(image);
        mCamera->trigger();

        if(image.isValid())
        {
            const bool tracking_status = tracker.track(image.getFrame());

        }
    }

    mCamera->close();
}


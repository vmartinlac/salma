#include <iostream>
#include <opencv2/core/version.hpp>
//#include <Eigen/Eigen>
#include <thread>
#include "FeatureDetector.h"
#include "BuildInfo.h"
#include "SLAMSystem.h"

std::unique_ptr<SLAMSystem> SLAMSystem::mInstance;

SLAMSystem* SLAMSystem::instance()
{
    if( bool(mInstance) == false )
    {
        mInstance.reset(new SLAMSystem());
    }

    return mInstance.get();
}

SLAMSystem::SLAMSystem()
{
}

SLAMSystem::~SLAMSystem()
{
}

bool SLAMSystem::initialize(int num_args, char** args)
{
    bool ok = true;

    printWelcomeMessage();
    
    ok = ok && parseCommandLineArguments(num_args, args);

    if(ok)
    {
        //TODO: load config.
    }

    if(ok)
    {
        mVideoReader.reset(new VideoReader());
        mVideoReader->setFileName("TODO");
    }

    return ok;
}

void SLAMSystem::finalize()
{
    mFirstFrame.reset();
    mLastFrame.reset();
    mMapPoints.clear();
    mVideoReader.reset();
}

bool SLAMSystem::parseCommandLineArguments(int num_args, char** args)
{
    /*
    Command line arguments:
    --slam-config=PATH
    --left-camera-calibration=PATH
    --right-camera-calibration=PATH
    --rig-calibration=PATH
    --video-input=PATH
    --reconstruction-output=PATH
    */

    // TODO: use some appropriate Qt class.

    return true;
}

void SLAMSystem::printWelcomeMessage()
{
    std::cout << std::endl;
    std::cout << " SALM" << std::endl; // TODO: make some ascii-art logo.
    std::cout << std::endl;
    std::cout << " Writen by Victor Martin Lac in 2018" << std::endl;
    std::cout << std::endl;
    std::cout << " Version: " << BuildInfo::getVersionMajor() << "." << BuildInfo::getVersionMinor() << "." << BuildInfo::getVersionRevision() << std::endl;
    std::cout << std::endl;
    std::cout << " Build date: " << BuildInfo::getCompilationDate() << std::endl;
    std::cout << " Compiler: " << BuildInfo::getCompilerName() << std::endl;
    std::cout << std::endl;
    std::cout << " OpenCV version: " << CV_VERSION << std::endl;
    std::cout << " Eigen version: " << EIGEN_WORLD_VERSION << "." << EIGEN_MAJOR_VERSION << "." << EIGEN_MINOR_VERSION << std::endl;
    std::cout << std::endl;
}

void SLAMSystem::computeFeatures(FramePtr frame)
{
    auto my_proc = [] (View* v)
    {
        FeatureDetector detector;
        detector.run(
            v->image,
            v->keypoints,
            v->descriptors);
    };

    std::thread t1(my_proc, &frame->views[0]);
    std::thread t2(my_proc, &frame->views[1]);

    t1.join();
    t2.join();
}

void SLAMSystem::track(FramePtr frame)
{
}

void SLAMSystem::map(FramePtr map)
{
}

void SLAMSystem::globalBundleAdjustment()
{
}

void SLAMSystem::run()
{
}



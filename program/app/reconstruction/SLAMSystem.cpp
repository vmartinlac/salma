#include <iostream>
#include <opencv2/core/version.hpp>
#include <Eigen/Eigen>
#include <memory>
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

    return ok;
}

void SLAMSystem::finalize()
{
}

bool SLAMSystem::parseCommandLineArguments(int num_args, char** args)
{
    /*
    Command line arguments:
    --left-camera-calibration=PATH
    --right-camera-calibration=PATH
    --rig-calibration=PATH
    --video-input=PATH
    --reconstruction-output=PATH
    */

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

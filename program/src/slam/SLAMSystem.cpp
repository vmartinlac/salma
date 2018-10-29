#include <QCoreApplication>
#include <QCommandLineParser>
#include <iostream>
#include <opencv2/core/version.hpp>
//#include <Eigen/Eigen>
#include <thread>
#include "VideoSystem.h"
#include "Image.h"
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
    const char* error_message = "";

    printWelcomeMessage();
    
    ok = ok && parseCommandLineArguments(num_args, args);

    if(ok)
    {
        ok = mVideo->open();
        error_message = "Could not open input video file!";
    }

    if(ok == false)
    {
        std::cerr << error_message << std::endl;
    }

    return ok;
}

void SLAMSystem::finalize()
{
    mVideo->close();

    mFirstFrame.reset();
    mCurrentFrame.reset();
    mMapPoints.clear();
    mVideo.reset();
}

bool SLAMSystem::parseCommandLineArguments(int num_args, char** args)
{
    QCommandLineParser parser;

    parser.addHelpOption();
    parser.addOption( QCommandLineOption("slam-config", "Path to SLAM configuration file", "FILE") );
    parser.addOption( QCommandLineOption("left-camera-calibration", "Path to left camera calibration file.", "FILE") );
    parser.addOption( QCommandLineOption("right-camera-calibration", "Path to right camera calibration file.", "FILE") );
    parser.addOption( QCommandLineOption("stereo-rig-calibration", "Path to rig calibration file.", "FILE") );
    parser.addOption( QCommandLineOption("video-input", "Path to video input file.", "FILE") );
    parser.addOption( QCommandLineOption("reconstruction-output", "Path to reconstruction output file.", "FILE") );

    parser.process(*QCoreApplication::instance());

    bool ok = true;
    const char* error_message = "";

    if(ok)
    {
        ok = ok && parser.isSet("left-camera-calibration");
        ok = ok && mCameraCalibration[0].loadFromFile( parser.value("left-camera-calibration").toStdString() );
        error_message = "Please set left camera calibration file!";
    }

    if(ok)
    {
        ok = ok && parser.isSet("right-camera-calibration");
        ok = ok && mCameraCalibration[1].loadFromFile( parser.value("right-camera-calibration").toStdString() );
        error_message = "Please set right camera calibration file!";
    }

    if(ok)
    {
        ok = ok && parser.isSet("stereo-rig-calibration");
        ok = ok && mStereoRigCalibration.loadFromFile( parser.value("stereo-rig-calibration").toStdString() );
        error_message = "Please set stereo rig calibration file!";
    }

    if(ok)
    {
        ok = ok && parser.isSet("slam-config");
        error_message = "Please set SLAM configuration file!";
    }

    // TODO: load config file.

    if(ok)
    {
        ok = ok && parser.isSet("video-input");
        error_message = "Please set video input file!";
    }

    if(ok)
    {
        mVideo = VideoSystem::instance()->createVideoSourceFromStereoRecording(parser.value("video-input").toStdString());
        ok = bool(mVideo);
        error_message = "Could not open video input file";
    }

    if(ok)
    {
        ok = ok && parser.isSet("reconstruction-output");
        error_message = "Please set reconstruction output file!";
    }

    // TODO: open output files.

    if(ok == false)
    {
        std::cerr << error_message << std::endl;
        exit(1);
    }

    return ok;
}

void SLAMSystem::printWelcomeMessage()
{
    std::cout << std::endl;
    std::cout << "   _____         _      __  __  " << std::endl;
    std::cout << "  / ____|  /\\   | |    |  \\/  | " << std::endl;
    std::cout << " | (___   /  \\  | |    | \\  / | " << std::endl;
    std::cout << "  \\___ \\ / /\\ \\ | |    | |\\/| | " << std::endl;
    std::cout << "  ____) / ____ \\| |____| |  | | " << std::endl;
    std::cout << " |_____/_/    \\_\\______|_|  |_| " << std::endl;
    std::cout << std::endl;
    std::cout << "Writen by Victor Martin Lac in 2018" << std::endl;
    std::cout << std::endl;
    std::cout << "Version: " << BuildInfo::getVersionMajor() << "." << BuildInfo::getVersionMinor() << "." << BuildInfo::getVersionRevision() << std::endl;
    std::cout << std::endl;
    std::cout << "Build date: " << BuildInfo::getCompilationDate() << std::endl;
    std::cout << "Compiler: " << BuildInfo::getCompilerName() << std::endl;
    std::cout << std::endl;
    std::cout << "OpenCV version: " << CV_VERSION << std::endl;
    std::cout << "Eigen version: " << EIGEN_WORLD_VERSION << "." << EIGEN_MAJOR_VERSION << "." << EIGEN_MINOR_VERSION << std::endl;
    std::cout << std::endl;
}

void SLAMSystem::computeFeatures(FramePtr frame)
{
    std::cout << "Computing features of frame " << frame->id << "." << std::endl;
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

    std::cout << "Number of features on left image: " << frame->views[0].keypoints.size() << std::endl;
    std::cout << "Number of features on right image: " << frame->views[1].keypoints.size() << std::endl;
}

void SLAMSystem::track(FramePtr frame)
{
    if(frame->id == 0)
    {
        frame->world_to_frame = Sophus::SE3d();
    }
    else
    {
        // TODO!
    }
}

void SLAMSystem::map(FramePtr map)
{
    ;
}

void SLAMSystem::globalBundleAdjustment()
{
}

void SLAMSystem::run()
{
    Image im;

    mVideo->trigger();
    mVideo->read(im);

    while(im.isValid())
    {
        mVideo->trigger();

        createNewFrame(im);

        std::cout << "=> Processing frame " << mCurrentFrame->id << "." << std::endl;

        computeFeatures(mCurrentFrame);

        track(mCurrentFrame);

        map(mCurrentFrame);

        mVideo->read(im);
    }
}

void SLAMSystem::createNewFrame(Image& image)
{
    FramePtr new_frame(new Frame());

    new_frame->previous_frame = mCurrentFrame;
    new_frame->id = (bool(mCurrentFrame)) ? mCurrentFrame->id+1 : 0;
    new_frame->timestamp = image.getTimestamp();
    new_frame->views[0].image = image.getFrame(0);
    new_frame->views[1].image = image.getFrame(1);

    mCurrentFrame.swap(new_frame);
}


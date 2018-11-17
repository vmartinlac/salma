#include <QCoreApplication>
#include <QCommandLineParser>
#include <opencv2/core/version.hpp>
#include <opencv2/calib3d.hpp>
#include <Eigen/Eigen>
#include <iostream>
#include <thread>
#include "VideoSystem.h"
#include "Image.h"
#include "FeatureDetector.h"
#include "BuildInfo.h"
#include "SLAMSystem.h"
#include "Debug.h"
#include "StereoMatcher.h"
#include "Misc.h"

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

void SLAMSystem::run(int num_args, char** args)
{
    if( initialize(num_args, args) == false )
    {
        throw std::runtime_error("initialization failed!");
    }

    Image im;

    mVideo->trigger();
    mVideo->read(im);

    /*
    Use opencv2/core/eigen.hpp
    */

    while(im.isValid())
    {
        mVideo->trigger();

        setCurrentFrame(im);

        std::cout << "=> Processing frame " << mCurrentFrame->id << "." << std::endl;

        cv::Mat rectified_left;
        cv::Mat rectified_right;
        cv::remap( mCurrentFrame->views[0].image, rectified_left, mRectification.camera[0].map0, mRectification.camera[0].map1, cv::INTER_LINEAR );
        cv::remap( mCurrentFrame->views[1].image, rectified_right, mRectification.camera[1].map0, mRectification.camera[1].map1, cv::INTER_LINEAR );

        cv::Ptr<cv::StereoMatcher> matcher = cv::StereoSGBM::create();
        //matcher->setBlockSize(15);

        const double kappa = 0.3;
        cv::resize(rectified_left, rectified_left, cv::Size(), kappa, kappa, cv::INTER_LINEAR);
        cv::resize(rectified_right, rectified_right, cv::Size(), kappa, kappa, cv::INTER_LINEAR);

        cv::Mat disparity;
        matcher->compute(rectified_left, rectified_right, disparity);

        /*
        Debug::stereoimshow( mCurrentFrame->views[0].image, mCurrentFrame->views[1].image );
        */
        Debug::stereoimshow(rectified_left, rectified_right);

        if( disparity.type() != CV_16S ) throw std::runtime_error("unexpected disparity image data type!");

        Debug::imshow(disparity);

        mVideo->read(im);
    }

    finalize();
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

    if(ok)
    {
        ok = ( mCameraCalibration[0]->image_size == mCameraCalibration[1]->image_size );
        error_message = "Image sizes should be equal";
    }

    if(ok)
    {
        const Sophus::SE3d left_to_right = mStereoRigCalibration->right_camera_to_world.inverse() * mStereoRigCalibration->left_camera_to_world;

        mRectification.R = Misc::eigenToMat( left_to_right.rotationMatrix() );
        mRectification.T = Misc::eigenToMat( left_to_right.translation() );

        cv::stereoRectify(
            mCameraCalibration[0]->calibration_matrix,
            mCameraCalibration[0]->distortion_coefficients,
            mCameraCalibration[1]->calibration_matrix,
            mCameraCalibration[1]->distortion_coefficients,
            mCameraCalibration[0]->image_size,
            mRectification.R,
            mRectification.T,
            mRectification.camera[0].R,
            mRectification.camera[1].R,
            mRectification.camera[0].P,
            mRectification.camera[1].P,
            mRectification.Q);

        for(int i=0; i<2; i++)
        {
            cv::initUndistortRectifyMap(
                mCameraCalibration[i]->calibration_matrix,
                mCameraCalibration[i]->distortion_coefficients,
                mRectification.camera[i].R,
                mRectification.camera[i].P,
                mCameraCalibration[i]->image_size,
                CV_32FC1,
                mRectification.camera[i].map0,
                mRectification.camera[i].map1 );
        }
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
    parser.addOption( QCommandLineOption("video-input", "Path to video input file.", "PATH") );
    parser.addOption( QCommandLineOption("reconstruction-output", "Path to reconstruction output file.", "PATH") );

    parser.process(*QCoreApplication::instance());

    bool ok = true;
    const char* error_message = "";

    if(ok)
    {
        mCameraCalibration[0].reset(new CameraCalibrationData());
        ok = ok && parser.isSet("left-camera-calibration");
        ok = ok && mCameraCalibration[0]->loadFromFile( parser.value("left-camera-calibration").toStdString() );
        error_message = "Please set left camera calibration file!";
    }

    if(ok)
    {
        mCameraCalibration[1].reset(new CameraCalibrationData());
        ok = ok && parser.isSet("right-camera-calibration");
        ok = ok && mCameraCalibration[1]->loadFromFile( parser.value("right-camera-calibration").toStdString() );
        error_message = "Please set right camera calibration file!";
    }

    if(ok)
    {
        mStereoRigCalibration.reset(new StereoRigCalibrationData());
        ok = ok && parser.isSet("stereo-rig-calibration");
        ok = ok && mStereoRigCalibration->loadFromFile( parser.value("stereo-rig-calibration").toStdString() );
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
        mVideo = VideoSystem::instance()->createVideoSourceFromFileStereo(parser.value("video-input").toStdString());
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

/*
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

void SLAMSystem::map(FramePtr f)
{
    std::vector< std::pair<int, int> > matches;

    StereoMatcher m;

    m.setLeftCameraCalibration( mCameraCalibration[0] );
    m.setRightCameraCalibration( mCameraCalibration[1] );
    m.setStereoRigCalibration( mStereoRigCalibration );

    m.match(f, matches);

    Debug::plotkeypointsandmatches(
        f->views[0].image,
        f->views[0].keypoints,
        f->views[1].image,
        f->views[1].keypoints,
        matches);
}
*/

void SLAMSystem::setCurrentFrame(Image& image)
{
    FramePtr new_frame(new Frame());

    new_frame->previous_frame = mCurrentFrame;
    new_frame->id = (bool(mCurrentFrame)) ? mCurrentFrame->id+1 : 0;
    new_frame->timestamp = image.getTimestamp();
    new_frame->views[0].image = image.getFrame(0);
    new_frame->views[1].image = image.getFrame(1);

    mCurrentFrame.swap(new_frame);
}


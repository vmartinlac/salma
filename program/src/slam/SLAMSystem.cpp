#include <QCoreApplication>
#include <QCommandLineParser>
#include <Eigen/Eigen>
#include <opencv2/core/version.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>
#include <thread>
#include "VideoSystem.h"
#include "Image.h"
#include "SLAMModulePyramid.h"
#include "SLAMModuleFeatures.h"
#include "SLAMModuleStereoMatcher.h"
#include "BuildInfo.h"
#include "SLAMSystem.h"
#include "Debug.h"
#include "Misc.h"

SLAMSystem::SLAMSystem()
{
}

SLAMSystem::~SLAMSystem()
{
}

bool SLAMSystem::initialize()
{
    const char* error = "";
    bool ret = true;

    QCommandLineParser parser;
    parser.setApplicationDescription("Perform SLAM on given video. Writen by Victor Martin Lac in 2018.");
    parser.addHelpOption();
    parser.addPositionalArgument("PROJECT_PATH", "Path to project root directory");

    parser.process(*QCoreApplication::instance());

    {
        mProject.reset(new SLAMProject());
        const std::string project_path = parser.positionalArguments().front().toStdString();
        ret = mProject->load(project_path.c_str());
        error = "Could not load project!";
    }

    if(ret)
    {
        mModulePyramid.reset(new SLAMModulePyramid(mProject));
        mModuleFeatures.reset(new SLAMModuleFeatures(mProject));
        mModuleStereoMatcher.reset(new SLAMModuleStereoMatcher(mProject));
    }

    if(ret == false)
    {
        std::cerr << error << std::endl;
    }

    return ret;
}

void SLAMSystem::run()
{
    Image im;
    VideoSourcePtr video;
    bool ok = true;

    printWelcomeMessage();

    if( ok )
    {
        ok = initialize();
    }

    if( ok )
    {
        video = mProject->getVideo();
        ok = bool(video);
    }

    if(ok)
    {
        ok = video->open();
    }

    if(ok)
    {
        video->trigger();
        video->read(im);
    }

    while(ok && im.isValid())
    {
        video->trigger();

        FramePtr new_frame(new Frame());

        new_frame->previous_frame = mCurrentFrame;
        new_frame->id = (bool(mCurrentFrame)) ? mCurrentFrame->id+1 : 0;
        new_frame->timestamp = im.getTimestamp();
        new_frame->views[0].image = im.getFrame(0);
        new_frame->views[1].image = im.getFrame(1);

        mCurrentFrame.swap(new_frame);

        std::cout << "Processing frame " << mCurrentFrame->id << std::endl;
        handleFrame(mCurrentFrame);

        video->read(im);
    }

    if( ok )
    {
        video->close();
        video.reset();

        finalize();
    }
}

void SLAMSystem::finalize()
{
    mModulePyramid.reset();
    mModuleFeatures.reset();
    mModuleStereoMatcher.reset();

    mProject.reset();

    mFirstFrame.reset();
    mCurrentFrame.reset();
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
}

void SLAMSystem::handleFrame(FramePtr frame)
{
    //Debug::imshow(frame->views[0].image);
}

    /*
    if(ok)
    {
        const Sophus::SE3d left_to_right = mStereoRigCalibration->right_camera_to_world.inverse() * mStereoRigCalibration->left_camera_to_world;

        mRectification.R;
        cv::eigen2cv( left_to_right.rotationMatrix(), mRectification.R );

        mRectification.T;
        cv::eigen2cv( left_to_right.translation(), mRectification.T );

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
    */


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


        /*
        cv::Ptr<cv::StereoBM> matcher = cv::StereoBM::create();
        cv::Mat rectified_left;
        cv::Mat rectified_right;
        cv::Mat disparity;

        matcher->setPreFilterType(cv::StereoBM::PREFILTER_NORMALIZED_RESPONSE);
        //matcher->setPreFilterSize(11);
        matcher->setMinDisparity(0);
        matcher->setNumDisparities(240);
        matcher->setBlockSize(101);
        matcher->setUniquenessRatio(2.0);
        //matcher->setTextureThreshold(0.95);

        mVideo->trigger();

        setCurrentFrame(im);

        std::cout << "=> Processing frame " << mCurrentFrame->id << "." << std::endl;

        // compute rectified images.

        cv::remap( mCurrentFrame->views[0].image, rectified_left, mRectification.camera[0].map0, mRectification.camera[0].map1, cv::INTER_LINEAR );
        cv::remap( mCurrentFrame->views[1].image, rectified_right, mRectification.camera[1].map0, mRectification.camera[1].map1, cv::INTER_LINEAR );


        // convert to grayscale.
        {
            cv::Mat tmp;

            cv::cvtColor(rectified_left, tmp, cv::COLOR_BGR2GRAY);
            cv::swap(rectified_left, tmp);

            cv::cvtColor(rectified_right, tmp, cv::COLOR_BGR2GRAY);
            cv::swap(rectified_right, tmp);
        }

        matcher->compute(rectified_left, rectified_right, disparity);
        Debug::imshow(rectified_left);
        Debug::imshow(disparity);

        if( disparity.type() != CV_16S ) throw std::runtime_error("unexpected disparity image data type!");

        */

    /*
    struct CameraRectification
    {
        cv::Mat R;
        cv::Mat P;
        cv::Mat map0;
        cv::Mat map1;
    };

    struct StereoRectification
    {
        CameraRectification camera[2];
        cv::Mat Q;
        cv::Mat R;
        cv::Mat T;
    };

    StereoRectification mRectification;
    */



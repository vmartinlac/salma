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
#include "BuildInfo.h"
#include "SLAMSystem.h"
#include "Debug.h"
#include "Misc.h"

SLAMSystem::SLAMSystem()
{
    mSkipTo = 0;
}

SLAMSystem::~SLAMSystem()
{
}

bool SLAMSystem::initialize()
{
    const char* error = "";
    bool ret = true;

    QCommandLineParser parser;
    {
        parser.setApplicationDescription("Perform SLAM on given video. Writen by Victor Martin Lac in 2018.");
        parser.addHelpOption();
        parser.addPositionalArgument("PROJECT_PATH", "Path to project root directory");

        QCommandLineOption skip_to_option("skip-to", "Index of first frame which will be processed", "FIRST_FRAME");
        skip_to_option.setDefaultValue("0");
        parser.addOption(skip_to_option);
    }

    if( ret )
    {
        ret = parser.parse(QCoreApplication::arguments());
        error = "Incorrect command line!";
    }

    if( ret )
    {
        ret = (parser.positionalArguments().size() == 1);
        error = "Incorrect command line!";
    }

    if(ret)
    {
        mSkipTo = parser.value("skip-to").toInt(&ret);
        error = "Incorrect value for skip-to argument!";
    }

    if(ret)
    {
        ret = (mSkipTo >= 0);
        error = "Incorrect value for skip-to argument!";
    }

    if(ret)
    {
        mProject.reset(new SLAMProject());
        const std::string project_path = parser.positionalArguments().front().toStdString();
        ret = mProject->load(project_path.c_str());
        error = "Could not load project!";
    }

    if(ret)
    {
        mModuleOpticalFlow.reset(new SLAMModuleOpticalFlow(mProject));
        mModuleAlignment.reset(new SLAMModuleAlignment(mProject));

        mModuleFeatures.reset(new SLAMModuleFeatures(mProject));
        mModuleStereoMatcher.reset(new SLAMModuleStereoMatcher(mProject));
        mModuleTriangulation.reset(new SLAMModuleTriangulation(mProject));

        mModuleDenseReconstruction.reset(new SLAMModuleDenseReconstruction(mProject));
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
    int count = 0;
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

        if( count >= mSkipTo )
        {
            FramePtr new_frame(new Frame());

            new_frame->previous_frame = mCurrentFrame;
            new_frame->id = count;
            new_frame->timestamp = im.getTimestamp();
            new_frame->views[0].image = im.getFrame(0);
            new_frame->views[1].image = im.getFrame(1);


            mCurrentFrame.swap(new_frame);

            //std::cout << "===> PROCESSING FRAME " << mCurrentFrame->id << " <===" <<std::endl;
            std::cout << "PROCESSING FRAME " << mCurrentFrame->id << std::endl;

            handleFrame(mCurrentFrame);

            std::cout << std::endl;
        }

        count++;

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
    mModuleOpticalFlow.reset();
    mModuleAlignment.reset();

    mModuleFeatures.reset();
    mModuleStereoMatcher.reset();
    mModuleTriangulation.reset();

    mModuleDenseReconstruction.reset();

    mProject.reset();

    mFirstFrame.reset();
    mCurrentFrame.reset();
}

void SLAMSystem::printWelcomeMessage()
{
    const std::string logo = BuildInfo::getAsciiLogo();

    std::cout << logo << std::endl;
    std::cout << "Version: " << BuildInfo::getVersionMajor() << "." << BuildInfo::getVersionMinor() << "." << BuildInfo::getVersionRevision() << std::endl;
    std::cout << std::endl;
    std::cout << "Writen by Victor Martin Lac in 2018" << std::endl;
    std::cout << std::endl;
    std::cout << "Build date: " << BuildInfo::getCompilationDate() << std::endl;
    std::cout << "Compiler: " << BuildInfo::getCompilerName() << std::endl;
    std::cout << std::endl;
}

void SLAMSystem::handleFrame(FramePtr frame)
{
    /////////////// TRACKING

    // optical flow.

    {
        std::cout << "   OPTICAL FLOW" << std::endl;

        mModuleOpticalFlow->run(frame);

        std::cout << "      Number of projections on left view: " << frame->views[0].projections.size() << std::endl;
        std::cout << "      Number of projections on right view: " << frame->views[1].projections.size() << std::endl;
    }

    // alignment.

    {
        std::cout << "   ALIGNMENT" << std::endl;

        mModuleAlignment->run(frame);

        std::cout << "      Alignment status: " << ( (frame->aligned_wrt_previous_frame) ? "ALIGNED" : "NOT ALIGNED" ) << std::endl;
        std::cout << "      Position: " << frame->frame_to_world.translation().transpose() << std::endl;
        std::cout << "      Attitude: " << frame->frame_to_world.unit_quaternion().coeffs().transpose() << std::endl;
    }

    /////////////// MAPPING

    // features detection.

    {
        std::cout << "   FEATURES DETECTION" << std::endl;

        mModuleFeatures->run(frame);

        std::cout << "      Num keypoints on left view: " << frame->views[0].keypoints.size() << std::endl;
        std::cout << "      Num keypoints on right view: " << frame->views[1].keypoints.size() << std::endl;
    }

    // stereo matching.

    {
        std::cout << "   STEREO MATCHING" << std::endl;

        mModuleStereoMatcher->match(frame);

        std::cout << "      Number of stereo matches: " << frame->stereo_matches.size() << std::endl;
    }

    // triangulation.

    {
        std::cout << "   TRIANGULATION" << std::endl;

        mModuleTriangulation->run(frame);
    }

    /////////////// RECONSTRUCTION

    // dense reconstruction.

    {
        std::cout << "   DENSE RECONSTRUCTION" << std::endl;

        mModuleDenseReconstruction->run(frame);
    }
}


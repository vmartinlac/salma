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
        mModuleFeatures.reset(new SLAMModuleFeatures(mProject));
        mModuleStereoMatcher.reset(new SLAMModuleStereoMatcher(mProject));
        mModuleTemporalMatcher.reset(new SLAMModuleTemporalMatcher(mProject));
        mModuleTriangulation.reset(new SLAMModuleTriangulation(mProject));
        mModuleAlignment.reset(new SLAMModuleAlignment(mProject));
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
    mModuleFeatures.reset();
    mModuleStereoMatcher.reset();
    mModuleTemporalMatcher.reset();
    mModuleTriangulation.reset();

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
    // detect features.

    {
        mModuleFeatures->run(frame);

        std::cout << "Num keypoints on left view: " << frame->views[0].keypoints.size() << std::endl;
        std::cout << "Num keypoints on right view: " << frame->views[1].keypoints.size() << std::endl;
    }

    // perform temporal matching.

    {
        mModuleTemporalMatcher->match(frame);

        auto op = [] (int count, const Track& t)
        {
            return ( t.anterior_match >= 0 ) ? count+1 : count;
        };

        const int count_left = std::accumulate( frame->views[0].tracks.begin(), frame->views[0].tracks.end(), 0, op );
        const int count_right = std::accumulate( frame->views[1].tracks.begin(), frame->views[1].tracks.end(), 0, op );

        std::cout << "Num temporal matches in left view: " << count_left << std::endl;
        std::cout << "Num temporal matches in right view: " << count_right << std::endl;
    }

    // perform stereo matching.

    {
        mModuleStereoMatcher->match(frame);

        int count = 0;

        for(int i=0; i<frame->views[0].tracks.size(); i++)
        {
            if( frame->views[0].tracks[i].stereo_match >= 0 )
            {
                count++;
            }
        }

        std::cout << "Number of stereo matches: " << count << std::endl;
    }

    // perform triangulation of new map points and alignment of current keyframe.

    {
        mModuleTriangulation->run(frame);
    }
}


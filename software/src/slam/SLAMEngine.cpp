#include <chrono>
#include "SLAMModule1Rectification.h"
#include "SLAMModule1Features.h"
#include "SLAMModule1TemporalMatcher.h"
#include "SLAMModule1Alignment.h"
#include "SLAMModule1KFS.h"
#include "SLAMModule2EKF.h"
#include "SLAMModule1LBA.h"
#include "SLAMModule1StereoMatcher.h"
#include "SLAMModule1Triangulation.h"
#include "SLAMModule1DenseReconstruction.h"
#include "SLAMEngine.h"

SLAMEngine::SLAMEngine()
{
}

SLAMEngine::~SLAMEngine()
{
}

bool SLAMEngine::initialize(
    StereoRigCalibrationPtr calibration,
    SLAMConfigurationPtr configuration)
{
    bool ok = true;

    if(ok)
    {
        mContext.reset(new SLAMContext);
        mContext->calibration = calibration;
        mContext->configuration = configuration;
        mContext->reconstruction.reset(new SLAMReconstruction());
        mContext->debug.reset(new SLAMDebug( mContext->configuration ));
        mContext->num_mappoints = 0;

        switch(configuration->pipeline)
        {

        case SLAM_PIPELINE1:
            mModules.emplace_back(new SLAMModule1Rectification(mContext));
            mModules.emplace_back(new SLAMModule1Features(mContext));
            mModules.emplace_back(new SLAMModule1TemporalMatcher(mContext));
            mModules.emplace_back(new SLAMModule1Alignment(mContext));
            mModules.emplace_back(new SLAMModule1KFS(mContext));
            mModules.emplace_back(new SLAMModule1LBA(mContext));
            mModules.emplace_back(new SLAMModule1StereoMatcher(mContext));
            mModules.emplace_back(new SLAMModule1Triangulation(mContext));
            mModules.emplace_back(new SLAMModule1DenseReconstruction(mContext));
            mNextModule = SLAM_MODULE1_RECTIFICATION;
            break;

        /*
        case SLAM_PIPELINE2:
            //mModules.emplace_back(new SLAMModule2OpticalFlow(mContext));
            //mModules.emplace_back(new SLAMModule2EKF(mContext));
            mNextModule = SLAM_MODULE2_OPTICALFLOW;
            break;
        */

        default:
            throw std::runtime_error("internal error");

        }
    }

    if(ok)
    {
        ok = mContext->debug->init();

        for(int i=0; ok && i<mModules.size(); i++)
        {
            ok = mModules[i]->init();
        }
    }

    return ok;
}

bool SLAMEngine::processFrame(int rank_in_recording, Image& image)
{
    if( image.isValid() )
    {
        if( image.getNumberOfFrames() != 2 ) throw std::runtime_error("incorrect number of frames");

        SLAMFramePtr curr_frame(new SLAMFrame());

        curr_frame->id = mContext->reconstruction->frames.size();
        curr_frame->rank_in_recording = rank_in_recording;
        curr_frame->timestamp = image.getTimestamp();


        //curr_frame->views[0].image = image.getFrame(0);
        //curr_frame->views[1].image = image.getFrame(1);

        cv::cvtColor(image.getFrame(0), curr_frame->views[0].image, cv::COLOR_BGR2GRAY);
        cv::cvtColor(image.getFrame(1), curr_frame->views[1].image, cv::COLOR_BGR2GRAY);

        if( curr_frame->views[0].image.type() != CV_8UC1 || curr_frame->views[1].image.type() != CV_8UC1 ) throw std::runtime_error("internal error");

#if SALMA_WITH_CUDA
        curr_frame->views[0].d_image.upload( curr_frame->views[0].image );
        curr_frame->views[1].d_image.upload( curr_frame->views[1].image );
#endif

        mContext->reconstruction->frames.push_back(curr_frame);

        std::cout << "PROCESSING FRAME " << curr_frame->id << std::endl;

        int milliseconds = 0;

        bool go_on = true;

        while(go_on)
        {
            std::chrono::time_point< std::chrono::steady_clock > t0 = std::chrono::steady_clock::now();

            std::vector<SLAMModulePtr>::iterator module_it = std::find_if(
                mModules.begin(),
                mModules.end(),
                [this] (SLAMModulePtr m) -> bool { return (m->id() == mNextModule); } );

            if(module_it == mModules.end()) throw std::runtime_error("internal error");

            SLAMModuleResult result = (**module_it)();

            go_on = (result.getEndPipeline() == false);
            mNextModule = result.getNextModule();

            std::chrono::time_point< std::chrono::steady_clock > t1 = std::chrono::steady_clock::now();

            const int module_milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(t1-t0).count();

            milliseconds += module_milliseconds;

            std::cout << "      Computation time: " << module_milliseconds << " ms" << std::endl;
        }

        std::cout << "   Frame computation time: " << milliseconds << " ms" << std::endl;
    }

    // free old images.

    {
        const int k = ( mContext->configuration->temporal_matcher.debug) ? mContext->configuration->temporal_matcher.num_previous_frames : 0;

        if( mContext->reconstruction->frames.size() >= 2 + k )
        {
            const int i = int( mContext->reconstruction->frames.size() ) - 2 - k;

            mContext->reconstruction->frames[i]->views[0].image.release(); // = cv::Mat();
            mContext->reconstruction->frames[i]->views[1].image.release(); // = cv::Mat();

#ifdef SALMA_WITH_CUDA
            mContext->reconstruction->frames[i]->views[0].d_image.release(); // = cv::Mat();
            mContext->reconstruction->frames[i]->views[1].d_image.release(); // = cv::Mat();
#endif
        }
    }

    return true;
}

bool SLAMEngine::finalize(SLAMReconstructionPtr& reconstruction)
{
    // build segments.

    mContext->reconstruction->buildSegments();

    // put reconstruction aside and clear all what was allocated.

    reconstruction = std::move(mContext->reconstruction);

    mContext.reset();
    mModules.clear();

    return true;
}


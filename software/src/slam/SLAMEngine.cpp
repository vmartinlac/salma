#include "SLAMModuleFeatures.h"
#include "SLAMModuleTemporalMatcher.h"
#include "SLAMModuleAlignment.h"
#include "SLAMModuleEKF.h"
#include "SLAMModuleLBA.h"
#include "SLAMModuleStereoMatcher.h"
#include "SLAMModuleTriangulation.h"
#include "SLAMModuleDenseReconstruction.h"
#include "SLAMEngine.h"

SLAMEngine::SLAMEngine()
{
}

SLAMEngine::~SLAMEngine()
{
}

bool SLAMEngine::initialize(
    StereoRigCalibrationDataPtr calibration,
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

        mModules.emplace_back(new SLAMModuleFeatures(mContext));
        mModules.emplace_back(new SLAMModuleTemporalMatcher(mContext));
        mModules.emplace_back(new SLAMModuleAlignment(mContext));
        //mModules.emplace_back(new SLAMModuleEKF(mContext));
        //mModules.emplace_back(new SLAMModuleLBA(mContext));
        mModules.emplace_back(new SLAMModuleStereoMatcher(mContext));
        mModules.emplace_back(new SLAMModuleTriangulation(mContext));
        //mModules.emplace_back(new SLAMModuleDenseReconstruction(mContext));
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
        curr_frame->views[0].image = image.getFrame(0);
        curr_frame->views[1].image = image.getFrame(1);

        mContext->reconstruction->frames.push_back(curr_frame);

        std::cout << "PROCESSING FRAME " << curr_frame->id << std::endl;

        for(SLAMModulePtr module : mModules)
        {
            (*module)();
        }
    }

    // free old images.

    {
        const int k = ( mContext->configuration->temporalmatcher_debug) ? mContext->configuration->temporalmatcher_num_previous_frames : 0;
        if( mContext->reconstruction->frames.size() >= 2 + k )
        {
            const int i = int( mContext->reconstruction->frames.size() ) - 2 - k;
            mContext->reconstruction->frames[i]->views[0].image = cv::Mat();
            mContext->reconstruction->frames[i]->views[1].image = cv::Mat();
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


#include "SLAMModuleFeatures.h"
#include "SLAMModuleTemporalMatcher.h"
#include "SLAMModuleAlignment.h"
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

        mModuleFeatures.reset(new SLAMModuleFeatures(mContext));
        mModuleTemporalMatcher.reset(new SLAMModuleTemporalMatcher(mContext));
        mModuleAlignment.reset(new SLAMModuleAlignment(mContext));
        mModuleStereoMatcher.reset(new SLAMModuleStereoMatcher(mContext));
        mModuleTriangulation.reset(new SLAMModuleTriangulation(mContext));
        mModuleDenseReconstruction.reset(new SLAMModuleDenseReconstruction(mContext));
    }

    if(ok)
    {
        ok = ok && mContext->debug->init();

        ok = ok && mModuleFeatures->init();
        ok = ok && mModuleTemporalMatcher->init();
        ok = ok && mModuleAlignment->init();
        ok = ok && mModuleStereoMatcher->init();
        ok = ok && mModuleTriangulation->init();
        ok = ok && mModuleDenseReconstruction->init();
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

        (*mModuleFeatures)();
        (*mModuleTemporalMatcher)();
        (*mModuleAlignment)();
        (*mModuleStereoMatcher)();
        (*mModuleTriangulation)();
        (*mModuleDenseReconstruction)();
    }

    // free old images.

    if( mContext->reconstruction->frames.size() >= 2 )
    {
        const int i = int( mContext->reconstruction->frames.size() ) - 2;
        mContext->reconstruction->frames[i]->views[0].image = cv::Mat();
        mContext->reconstruction->frames[i]->views[1].image = cv::Mat();
    }

    return true;
}

bool SLAMEngine::finalize(SLAMReconstructionPtr& reconstruction)
{
    // assign map point ids.

    int num_mappoints = 0;
    for( SLAMFramePtr& f : mContext->reconstruction->frames )
    {
        for(int i=0; i<2; i++)
        {
            for( SLAMTrack& t : f->views[i].tracks )
            {
                if(t.mappoint && t.mappoint->id < 0)
                {
                    t.mappoint->id = num_mappoints;
                    num_mappoints++;
                }
            }
        }
    }

    // build segments.

    mContext->reconstruction->buildSegments();

    // put reconstruction aside and clear all what was allocated.

    reconstruction = std::move(mContext->reconstruction);

    mContext.reset();
    mModuleFeatures.reset();
    mModuleTemporalMatcher.reset();
    mModuleAlignment.reset();
    mModuleStereoMatcher.reset();
    mModuleTriangulation.reset();
    mModuleDenseReconstruction.reset();

    return true;
}


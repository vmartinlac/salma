#include "FinitePriorityQueue.h"
#include "SLAMModule1Rectification.h"

SLAMModule1Rectification::SLAMModule1Rectification(SLAMContextPtr con) :
    SLAMModule(SLAM_MODULE1_RECTIFICATION, con)
{
}

SLAMModule1Rectification::~SLAMModule1Rectification()
{
}

bool SLAMModule1Rectification::init()
{
    /*
    SLAMContextPtr con = context();

    const double scale_factor = con->configuration->features.scale_factor;
    const int min_width = con->configuration->features.min_width;
    const int max_features = con->configuration->features.max_features;
    const int patch_size = con->configuration->features.patch_size;
    const int fast_threshold = con->configuration->features.fast_threshold;

    const int reference_image_width = con->calibration->cameras[0].image_size.width;

    const int num_levels = std::floor( std::log(double(reference_image_width)/double(min_width)) / std::log(scale_factor) );

    mFeature2d = cv::ORB::create();

    mFeature2d->setScoreType(cv::ORB::HARRIS_SCORE);
    mFeature2d->setScaleFactor(scale_factor);
    mFeature2d->setNLevels(num_levels);
    mFeature2d->setFirstLevel(0);
    mFeature2d->setPatchSize(patch_size);
    mFeature2d->setFastThreshold(fast_threshold);
    mFeature2d->setMaxFeatures(max_features);
    */

    return true;
}

SLAMModuleResult SLAMModule1Rectification::operator()()
{
    std::cout << "   RECTIFICATION" << std::endl;

    SLAMReconstructionPtr reconstr = context()->reconstruction;

    if( reconstr->frames.empty() ) throw std::runtime_error("internal error");

    SLAMFramePtr frame = reconstr->frames.back();

    return SLAMModuleResult(false, SLAM_MODULE1_FEATURES);
}


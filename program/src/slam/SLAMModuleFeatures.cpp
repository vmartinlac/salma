#include "FinitePriorityQueue.h"
#include "SLAMModuleFeatures.h"

SLAMModuleFeatures::SLAMModuleFeatures(SLAMProjectPtr project) : SLAMModule(project)
{
    const double scale_factor = project->getParameterReal("features_scale_factor", 1.1);
    const int min_width = project->getParameterInteger("features_min_width", 160);
    const int max_features = project->getParameterInteger("features_max_features", 500);
    const int patch_size = project->getParameterInteger("features_patch_size", 31);
    const int fast_threshold = project->getParameterInteger("features_fast_threshold", 10);

    const int num_levels = std::floor( std::log(double(project->getLeftCameraCalibration()->image_size.width)/double(min_width)) / std::log(scale_factor) );

    mFeature2d = cv::ORB::create();

    mFeature2d->setScoreType(cv::ORB::HARRIS_SCORE);
    mFeature2d->setScaleFactor(scale_factor);
    mFeature2d->setNLevels(num_levels);
    mFeature2d->setFirstLevel(0);
    mFeature2d->setPatchSize(patch_size);
    mFeature2d->setFastThreshold(fast_threshold);
    mFeature2d->setMaxFeatures(max_features);
}

void SLAMModuleFeatures::run(FramePtr frame)
{
    for(int i=0; i<2; i++)
    {
        runOnView(frame->views[i].image, frame->views[i].keypoints, frame->views[i].descriptors);
        frame->views[i].tracks.resize( frame->views[i].keypoints.size() );
    }
}

void SLAMModuleFeatures::runOnView(cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors)
{
    mFeature2d->detectAndCompute(image, cv::Mat(), keypoints, descriptors);
}


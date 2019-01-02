#include "FinitePriorityQueue.h"
#include "SLAMModuleFeatures.h"

//#define DEBUG_SHOW_FEATURES

SLAMModuleFeatures::SLAMModuleFeatures(SLAMContextPtr con) : SLAMModule(con)
{
}

SLAMModuleFeatures::~SLAMModuleFeatures()
{
}

bool SLAMModuleFeatures::init()
{
    SLAMContextPtr con = context();

    const double scale_factor = con->configuration->features_scale_factor;
    const int min_width = con->configuration->features_min_width;
    const int max_features = con->configuration->features_max_features;
    const int patch_size = con->configuration->features_patch_size;
    const int fast_threshold = con->configuration->features_fast_threshold;

    const int reference_image_width = con->calibration->cameras[0].calibration->image_size.width;

    const int num_levels = std::floor( std::log(double(reference_image_width)/double(min_width)) / std::log(scale_factor) );

    mFeature2d = cv::ORB::create();

    mFeature2d->setScoreType(cv::ORB::HARRIS_SCORE);
    mFeature2d->setScaleFactor(scale_factor);
    mFeature2d->setNLevels(num_levels);
    mFeature2d->setFirstLevel(0);
    mFeature2d->setPatchSize(patch_size);
    mFeature2d->setFastThreshold(fast_threshold);
    mFeature2d->setMaxFeatures(max_features);

    return true;
}

void SLAMModuleFeatures::operator()()
{
    SLAMReconstructionPtr reconstr = context()->reconstruction;

    if( reconstr->frames.empty() ) throw std::runtime_error("internal error");

    SLAMFramePtr frame = reconstr->frames.back();

    for(int i=0; i<2; i++)
    {
        runOnView(frame->views[i].image, frame->views[i].keypoints, frame->views[i].descriptors);
    }

#ifdef DEBUG_SHOW_FEATURES
    Debug::stereoimshow(
        frame->views[0].image,
        frame->views[1].image,
        frame->views[0].keypoints,
        frame->views[1].keypoints );
#endif
}

void SLAMModuleFeatures::runOnView(cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors)
{
    mFeature2d->detectAndCompute(image, cv::Mat(), keypoints, descriptors);
}

/*
const int N = keypoints.size();

const int M = 400;
if( N <= M ) return;

std::vector<int> selection(N);
for(int i=0; i<N; i++)
{
    selection[i] = i;
}

std::sort( selection.begin(), selection.end(), [&keypoints] (int i, int j) { return keypoints[i].response > keypoints[j].response; } );

std::vector<double> radius(N);

for(int i=0; i<N; i++)
{
    double value = std::numeric_limits<double>::max();

    const double kappa = 0.9;

    for(int j=0; j<i && keypoints[selection[i]].response < kappa*keypoints[selection[j]].response; j++)
    {
        value = std::min(value, cv::norm( keypoints[selection[i]].pt - keypoints[selection[j]].pt ));
    }

    radius[selection[i]] = value;
}

std::vector<double> sorted_radii(N);
std::copy(radius.begin(), radius.end(), sorted_radii.begin());
std::sort(sorted_radii.begin(), sorted_radii.end());

std::vector<cv::KeyPoint> fkeypoints(M);
cv::Mat fdescriptors(M, descriptors.cols, descriptors.type());

int k = 0;
for(int i=0; k < M && i<N; i++)
{
    if( radius[i] >= sorted_radii[M] )
    {
        fkeypoints[k] = keypoints[i];
        fdescriptors.row(k) = descriptors.row(i);
        k++;
    }
}

fkeypoints.swap(keypoints);
descriptors = std::move(fdescriptors);
*/


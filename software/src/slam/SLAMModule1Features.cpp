#if SALMA_WITH_CUDA
#include <opencv2/core/cuda.hpp>
//#include <opencv2/cudastereo.hpp>
//#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudafeatures2d.hpp>
#endif
#include "FinitePriorityQueue.h"
#include "SLAMModule1Features.h"

SLAMModule1Features::SLAMModule1Features(SLAMContextPtr con) :
    SLAMModule(SLAM_MODULE1_FEATURES, con)
{
}

SLAMModule1Features::~SLAMModule1Features()
{
}

bool SLAMModule1Features::init()
{
    SLAMContextPtr con = context();

    mScaleFactor = con->configuration->features.scale_factor;

    const int min_width = con->configuration->features.min_width;
    const int max_features = con->configuration->features.max_features;
    const int patch_size = con->configuration->features.patch_size;
    const int fast_threshold = con->configuration->features.fast_threshold;

    const int reference_image_width = con->calibration->cameras[0].image_size.width;

    mNumLevels = std::floor( std::log(double(reference_image_width)/double(min_width)) / std::log(mScaleFactor) );

#ifdef SALMA_WITH_CUDA
    mFeature2d = cv::cuda::ORB::create();
#else
    mFeature2d = cv::ORB::create();
#endif

    mFeature2d->setScoreType(cv::ORB::HARRIS_SCORE);
    mFeature2d->setScaleFactor(mScaleFactor);
    mFeature2d->setNLevels(mNumLevels);
    mFeature2d->setFirstLevel(0);
    mFeature2d->setPatchSize(patch_size);
    mFeature2d->setFastThreshold(fast_threshold);
    mFeature2d->setMaxFeatures(max_features);

    return true;
}

SLAMModuleResult SLAMModule1Features::operator()()
{
    std::cout << "   FEATURES DETECTION" << std::endl;

    SLAMReconstructionPtr reconstr = context()->reconstruction;

    if( reconstr->frames.empty() ) throw std::runtime_error("internal error");

    SLAMFramePtr frame = reconstr->frames.back();

    for(int i=0; i<2; i++)
    {
        processView(frame->views[i]);
    }

    if( context()->configuration->features.debug )
    {
        for(int i=0; i<2; i++)
        {
            cv::Mat outimg;

            cv::drawKeypoints(
                frame->views[i].image,
                frame->views[i].keypoints,
                outimg);

            context()->debug->saveImage(frame->id, "FEATURES_view"+std::to_string(i)+".png", outimg);
        }
    }

    std::cout << "      Num keypoints on left view: " << frame->views[0].keypoints.size() << std::endl;
    std::cout << "      Num keypoints on right view: " << frame->views[1].keypoints.size() << std::endl;

    return SLAMModuleResult(false, SLAM_MODULE1_TEMPORALMATCHER);
}

void SLAMModule1Features::processView(SLAMView& v)
{
    // Interest point detection with Shi-Tomasi.

    {
        cv::Ptr<cv::GFTTDetector> gftt = cv::GFTTDetector::create(2000);

        for(int octave = 0; octave<mNumLevels; octave++)
        {
            const double scale = std::pow(mScaleFactor, static_cast<double>(octave));

            cv::Mat level;

            if(octave == 0)
            {
                level = v.image;
            }
            else
            {
                cv::resize(v.image, level, cv::Size(), 1.0/scale, 1.0/scale);
            }

            std::vector<cv::KeyPoint> keypoints;

            gftt->detect(level, keypoints);

            auto proc = [octave, scale] (const cv::KeyPoint& kp)
            {
                cv::KeyPoint ret = kp;
                ret.pt.x *= scale;
                ret.pt.y *= scale;
#ifdef SALMA_WITH_CUDA
                ret.octave = 0;
#else
                ret.octave = octave;
#endif
                return ret;
            };

            std::transform(keypoints.begin(), keypoints.end(), std::back_inserter(v.keypoints), proc);
        }
    }

    // interest point selection.

    uniformize(v.keypoints);

    // compute ORB descriptors.

    {
#ifdef SALMA_WITH_CUDA
        cv::cuda::GpuMat d_descriptors;
        mFeature2d->compute( v.d_image, v.keypoints, d_descriptors );
        d_descriptors.download(v.descriptors);
#else
        mFeature2d->compute( v.image, v.keypoints, v.descriptors );
#endif
    }

    v.tracks.resize( v.keypoints.size() );
}

void SLAMModule1Features::uniformize(std::vector<cv::KeyPoint>& keypoints)
{
    const int N = keypoints.size();

    const int M = context()->configuration->features.max_features;

    if( N > M )
    {
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

            const double kappa = 1.0;

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

        int k = 0;
        for(int i=0; k < M && i<N; i++)
        {
            if( radius[selection[i]] >= sorted_radii[M] )
            {
                fkeypoints[k] = keypoints[selection[i]];
                k++;
            }
        }

        if(k != M) throw std::runtime_error("internal error");

        fkeypoints.swap(keypoints);
    }
}


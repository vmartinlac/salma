#include "FinitePriorityQueue.h"
#include "SLAMModuleFeatures.h"

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
    std::cout << "   FEATURES DETECTION" << std::endl;

    SLAMReconstructionPtr reconstr = context()->reconstruction;

    if( reconstr->frames.empty() ) throw std::runtime_error("internal error");

    SLAMFramePtr frame = reconstr->frames.back();

    for(int i=0; i<2; i++)
    {
        processView(frame->views[i]);
    }

    if( context()->configuration->features_debug )
    {
        for(int i=0; i<2; i++)
        {
            cv::Mat outimg;

            cv::drawKeypoints(
                frame->views[i].image,
                frame->views[i].keypoints,
                outimg);

            context()->debug->saveImage(frame->id, "FEATURES_view"+std::to_string(i), outimg);
        }
    }

    std::cout << "      Num keypoints on left view: " << frame->views[0].keypoints.size() << std::endl;
    std::cout << "      Num keypoints on right view: " << frame->views[1].keypoints.size() << std::endl;
}

void SLAMModuleFeatures::processView(SLAMView& v)
{

    //if( context()->configuration->features_uniformize )
    {
        cv::Ptr<cv::GFTTDetector> gftt = cv::GFTTDetector::create(2000);
        bool go_on = true;
        cv::Mat level = v.image;
        double scale = 1.0;
        int octave = 0;

        while(go_on)
        {
            auto proc = [octave, scale] (const cv::KeyPoint& kp)
            {
                cv::KeyPoint ret = kp;
                ret.pt.x *= scale;
                ret.pt.y *= scale;
                ret.octave = octave;
                return ret;
            };

            std::vector<cv::KeyPoint> keypoints;

            gftt->detect(level, keypoints);

            std::transform(keypoints.begin(), keypoints.end(), std::back_inserter(v.keypoints), proc);

            scale *= context()->configuration->features_scale_factor;

            octave++;

            go_on = ( v.image.cols / scale > context()->configuration->features_min_width );

            if(go_on)
            {
                cv::resize(v.image, level, cv::Size(), 1.0/scale, 1.0/scale);
            }
        }

        uniformize(v.keypoints);

        mFeature2d->compute( v.image, v.keypoints, v.descriptors );
    }
    //

    mFeature2d->compute( v.image, v.keypoints, v.descriptors );
    //mFeature2d->detectAndCompute( v.image, cv::Mat(), v.keypoints, v.descriptors );

    v.tracks.resize( v.keypoints.size() );
}

void SLAMModuleFeatures::uniformize(std::vector<cv::KeyPoint>& keypoints)
{
    const int N = keypoints.size();

    const int M = context()->configuration->features_max_features;

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


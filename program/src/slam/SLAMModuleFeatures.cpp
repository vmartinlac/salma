#include "FinitePriorityQueue.h"
#include "SLAMModuleFeatures.h"

SLAMModuleFeatures::SLAMModuleFeatures(SLAMProjectPtr project) : SLAMModule(project)
{
}

void SLAMModuleFeatures::run(cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors)
{
    buildPyramid(image);
    detectAllKeyPoints();
    binKeyPoints(false);
    binKeyPoints(true);
    describeKeyPoints();

    keypoints.swap(mKeyPoints);
    descriptors = mDescriptors;

    mPyramid.clear();
    mKeyPoints.clear();
    mDescriptors = cv::Mat();
}

void SLAMModuleFeatures::buildPyramid(cv::Mat& image)
{
    const int min_width = 150;
    const double scale_factor = 0.7;
    const int levels = std::max<int>(
        1,
        int(std::floor(std::log(double(min_width)/double(image.cols))/std::log(scale_factor))) );

    //std::cout << "Number of levels: " << levels << std::endl;

    mPyramid.resize(levels);

    mPyramid.front().image = image;
    mPyramid.front().scale = 1.0;

    for(int i=1; i<levels; i++)
    {
        mPyramid[i].scale = mPyramid[i-1].scale * scale_factor;

        cv::resize(
            mPyramid[i-1].image,
            mPyramid[i].image,
            cv::Size(),
            scale_factor,
            scale_factor);
    }
}

void SLAMModuleFeatures::detectAllKeyPoints()
{
    mKeyPoints.clear();

    cv::Ptr<cv::GFTTDetector> gftt = cv::GFTTDetector::create();

    for(int i=0; i<mPyramid.size(); i++)
    {
        std::vector<cv::KeyPoint> kpts;
        gftt->detect(mPyramid[i].image, kpts);

        for(cv::KeyPoint& kp : kpts)
        {
            kp.pt.x /= float(mPyramid[i].scale);
            kp.pt.y /= float(mPyramid[i].scale);
            kp.octave = i;
        }

        mKeyPoints.insert(mKeyPoints.end(), kpts.begin(), kpts.end());
    }
}

void SLAMModuleFeatures::binKeyPoints(bool second)
{
    const int cell_length = 20;
    const int cell_delta = (second) ? cell_length/2 : 0;

    const int num_bins_x = 2 + mPyramid.front().image.cols/cell_length;
    const int num_bins_y = 2 + mPyramid.front().image.rows/cell_length;

    /*
    const int cell_min_length = 30;
    const int num_bins_x = std::max<int>(1, mPyramid.front().image.cols/cell_min_length);
    const int num_bins_y = std::max<int>(1, mPyramid.front().image.rows/cell_min_length);
    */

    typedef FinitePriorityQueueF<size_t,double,1> FPQ;

    std::vector<FPQ> bins( num_bins_x*num_bins_y );

    /*
    for( FPQ& fpq : bins )
    {
      fpq.reset(max_keypoints_per_cell);
    }
    */

    for(size_t i=0; i<mKeyPoints.size(); i++)
    {
        const int a = (int(mKeyPoints[i].pt.x) + cell_delta) / cell_length;
        const int b = (int(mKeyPoints[i].pt.y) + cell_delta) / cell_length;
        //const int b = int(mKeyPoints[i].pt.y) * num_bins_y / mPyramid.front().image.rows;
        const int bin = b * num_bins_x + a;

        bins[bin].push( i, mKeyPoints[i].response );
    }

    std::vector<cv::KeyPoint> newkpts;
    newkpts.reserve(mKeyPoints.size());

    for(size_t i=0; i<bins.size(); i++)
    {
        for(size_t j : bins[i])
        {
            newkpts.push_back(std::move(mKeyPoints[j]));
        }
    }

    mKeyPoints.swap(newkpts);
}

void SLAMModuleFeatures::describeKeyPoints()
{
    cv::Ptr<cv::ORB> orb = cv::ORB::create();
    orb->compute(mPyramid.front().image, mKeyPoints, mDescriptors);
}


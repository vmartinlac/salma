
#pragma once

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#ifdef SALMA_WITH_CUDA
#include <opencv2/cudafeatures2d.hpp>
#endif
#include <utility>
#include <cmath>
#include "SLAMDataStructures.h"
#include "SLAMModule.h"

class SLAMModule1Features : public SLAMModule
{
public:

    SLAMModule1Features(SLAMContextPtr con);
    ~SLAMModule1Features() override;

    bool init() override;
    SLAMModuleResult operator()() override;

protected:

    void processView(SLAMView& v);
    void processViewsSimple(SLAMView& left, SLAMView& right);
    void uniformize(std::vector<cv::KeyPoint>& keypoints);

protected:

    int mFirstLevel;
    int mNumLevels;
    double mScaleFactor;
    int mMaxFeatures;

#ifdef SALMA_WITH_CUDA
    cv::Ptr<cv::cuda::ORB> mFeature2d;
#else
    cv::Ptr<cv::ORB> mFeature2d;
#endif

};


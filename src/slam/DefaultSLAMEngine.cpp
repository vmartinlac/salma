#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include "DefaultSLAMEngine.h"
#include "Image.h"

SLAMEngine* SLAMEngine::createDefaultSLAMEngine()
{
    return new DefaultSLAMEngine();
}

void DefaultSLAMEngine::initialize()
{
    ;
}

void DefaultSLAMEngine::processNextView(Image* image)
{
    cv::Mat greylevel;
    cv::cvtColor(image->frame(), greylevel, cv::COLOR_RGB2GRAY);

    cv::Ptr<cv::Feature2D> detector = cv::GFTTDetector::create(600, 0.01, 10);

    std::vector<cv::KeyPoint> key_points;
    detector->detect(greylevel, key_points);

    cv::Ptr<cv::Feature2D> descriptor = cv::xfeatures2d::BriefDescriptorExtractor::create();

    cv::Mat corners;
    cv::goodFeaturesToTrack(greylevel, corners, 500, 0.01, 20);
}

DefaultSLAMEngine::DefaultSLAMEngine()
{
    ;
}

DefaultSLAMEngine::~DefaultSLAMEngine()
{
    ;
}

std::string DefaultSLAMEngine::name()
{
    return "SLAM engine";
}

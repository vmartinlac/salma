#include <opencv2/imgproc.hpp>
#include <opencv2/video.hpp>
#include <opencv2/features2d.hpp>
#include "DefaultSLAMEngine.h"
#include "Image.h"

SLAMEngine* SLAMEngine::createDefaultSLAMEngine()
{
    return new DefaultSLAMEngine();
}

void DefaultSLAMEngine::initialize()
{
    m_prev_pose.create(4, 4, CV_32FC1);
    //m_prev_pose

    if( m_prev_image != nullptr )
    {
        delete m_prev_image;
    }
    m_prev_image = nullptr;
}

void DefaultSLAMEngine::processNextView(Image* image)
{
    /*
    cv::Ptr<cv::Feature2D> detector = cv::GFTTDetector::create(600, 0.01, 10);

    std::vector<cv::KeyPoint> key_points;
    detector->detect(greylevel, key_points);

    cv::Ptr<cv::Feature2D> descriptor = cv::xfeatures2d::BriefDescriptorExtractor::create();
    */

    // convert both previous and current image to greylevel.
    // (we could save the greylevel to avoid recomputation).

    cv::Mat prev_greylevel;
    cv::cvtColor(m_prev_image->frame(), prev_greylevel, cv::COLOR_RGB2GRAY);

    cv::Mat curr_greylevel;
    cv::cvtColor(image->frame(), curr_greylevel, cv::COLOR_RGB2GRAY);

    // compute GFTT.

    cv::Mat prev_corners;
    cv::goodFeaturesToTrack(prev_greylevel, prev_corners, 500, 0.01, 20);
    cv::cornerSubPix(prev_greylevel, prev_corners, cv::Size(8, 8), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::MAX_ITER, 100, 1.0e-5));

    // compute sparse optical flow.

    cv::Mat curr_corners;
    cv::Mat lkstatus;
    cv::Mat lkerr;
    cv::calcOpticalFlowPyrLK(prev_greylevel, curr_greylevel, prev_corners, curr_corners, lkstatus, lkerr);

    // compute ego-motion.

    // TODO
}

DefaultSLAMEngine::DefaultSLAMEngine()
{
    m_prev_image = nullptr;
    m_prev_pose.create(4, 4, CV_32FC1);
}

DefaultSLAMEngine::~DefaultSLAMEngine()
{
    if( m_prev_image != nullptr )
    {
        delete m_prev_image;
    }
}

std::string DefaultSLAMEngine::name()
{
    return "SLAM engine";
}

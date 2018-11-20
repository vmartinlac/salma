#include "SLAMModuleTemporalMatcher.h"
#include "Debug.h"

#define DEBUG_SHOW_OPTICAL_FLOW

SLAMModuleTemporalMatcher::SLAMModuleTemporalMatcher(SLAMProjectPtr project) : SLAMModule(project)
{
    mCheckSymmetry = project->getParameterBoolean("temporal_matcher_check_symmetry", true);
    mCheckLowe = project->getParameterBoolean("temporal_matcher_check_lowe", true);
    mLoweRatio = project->getParameterBoolean("temporal_matcher_lowe_ratio", 0.85);
    mCheckOpticalFlow = project->getParameterBoolean("temporal_matcher_check_optical_flow", true);
    mOpticalFlowRadius = project->getParameterBoolean("temporal_matcher_optical_flow_radius", 30);
    mCheckOctave = project->getParameterBoolean("temporal_matcher_check_octave", false);

    const int size = project->getParameterInteger("temporal_matcher_win_size", 21);

    const int min_width = 200;
    const int max_level = std::floor( std::log(double(project->getLeftCameraCalibration()->image_size.width)/double(min_width)) / std::log(2.0) );

    mLKT = cv::SparsePyrLKOpticalFlow::create();

    mLKT->setWinSize(cv::Size(size, size));
    mLKT->setMaxLevel(max_level);
}

void SLAMModuleTemporalMatcher::match(FramePtr frame)
{
    frame->views[0].tracks.resize( frame->views[0].keypoints.size() );
    frame->views[1].tracks.resize( frame->views[1].keypoints.size() );

    if( frame->previous_frame )
    {
        processView(frame, 0);
        processView(frame, 1);
    }
}

void SLAMModuleTemporalMatcher::processView(FramePtr frame, int view)
{
    std::vector<cv::Point2f> previous_points;
    cv::KeyPoint::convert( frame->previous_frame->views[view].keypoints, previous_points );

    std::vector<cv::Point2f> next_points;
    std::vector<uint8_t> status;

    mLKT->calc(
        frame->previous_frame->views[view].image,
        frame->views[view].image,
        previous_points,
        next_points,
        status);

#ifdef DEBUG_SHOW_OPTICAL_FLOW
    {
        cv::Mat image = frame->views[view].image.clone();

        for(int i=0; i<previous_points.size(); i++)
        {
            cv::line(image, previous_points[i], next_points[i], cv::Scalar(0, 255, 0), 4);
            cv::circle(image, next_points[i], 3, cv::Scalar(0, 255, 0), -1);
        }

        Debug::imshow(image);
    }
#endif

    // TODO!
}


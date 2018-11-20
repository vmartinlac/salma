#include "FinitePriorityQueue.h"
#include "SLAMModuleTemporalMatcher.h"
#include "Debug.h"

//#define DEBUG_SHOW_OPTICAL_FLOW
//#define DEBUG_SHOW_MATCHES

SLAMModuleTemporalMatcher::SLAMModuleTemporalMatcher(SLAMProjectPtr project) : SLAMModule(project)
{
    mCheckSymmetry = project->getParameterBoolean("temporal_matcher_check_symmetry", true);
    mCheckLowe = project->getParameterBoolean("temporal_matcher_check_lowe", true);
    mLoweRatio = project->getParameterReal("temporal_matcher_lowe_ratio", 0.85);
    mCheckOctave = project->getParameterBoolean("temporal_matcher_check_octave", false);
    mPredictionRadius = project->getParameterReal("temporal_matcher_prediction_radius", 30.0);

    const int size = project->getParameterInteger("temporal_matcher_window_size", 21);

    const int min_width = 150;
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
    View& previous_view = frame->previous_frame->views[view];
    View& current_view = frame->views[view];

    const int N = previous_view.keypoints.size();

    std::vector<cv::Point2f> opticalflow_pre;
    cv::KeyPoint::convert( previous_view.keypoints, opticalflow_pre );

    std::vector<cv::Point2f> opticalflow_post;
    std::vector<uint8_t> opticalflow_status;

    mLKT->calc(
        previous_view.image,
        current_view.image,
        opticalflow_pre,
        opticalflow_post,
        opticalflow_status);

#ifdef DEBUG_SHOW_OPTICAL_FLOW
    {
        cv::Mat image = current_view.image;

        for(int i=0; i<N; i++)
        {
            cv::line(image, opticalflow_pre[i], opticalflow_post[i], cv::Scalar(0, 255, 0), 4);
            cv::circle(image, opticalflow_post[i], 3, cv::Scalar(0, 255, 0), -1);
        }

        Debug::imshow(image);
    }
#endif

    for(int i=0; i<N; i++)
    {
        if( opticalflow_status[i] > 0 )
        {
            const int j = matchKeyPoint(i, opticalflow_post[i], previous_view, current_view, mCheckSymmetry);

            if( j >= 0 )
            {
                if( current_view.tracks[j].match_in_previous_frame >= 0 )
                {
                    std::cerr << "Keypoint already matched!" << std::endl;
                }

                current_view.tracks[j].match_in_previous_frame = i;
            }
        }
    }

#ifdef DEBUG_SHOW_MATCHES
    {
        std::vector< std::pair<int,int> > matches;

        for(int j=0; j<current_view.keypoints.size(); j++)
        {
            const int i = current_view.tracks[j].match_in_previous_frame;

            if( i >= 0 )
            {
                matches.push_back( std::pair<int,int>(i, j) );
            }
        }
        std::cout << "M " << view << " " << matches.size() << std::endl;

        Debug::stereoimshow(
            previous_view.image,
            current_view.image,
            previous_view.keypoints,
            current_view.keypoints,
            matches);
    }
#endif
}

int SLAMModuleTemporalMatcher::matchKeyPoint(int i, const cv::Point2f& prediction, const View& from, const View& to, bool check_symmetry)
{
    FinitePriorityQueueF<int, double, 2> queue;

    for(int j=0; j<to.keypoints.size(); j++)
    {
        bool ok = true;

        if(ok && mCheckOctave)
        {
            ok = ( from.keypoints[i].octave == to.keypoints[j].octave );
        }

        if(ok)
        {
            const double distance = cv::norm(
                from.descriptors.row(i),
                to.descriptors.row(j));

            queue.push( j, -distance);
        }
    }

    int ret = -1;

    if( queue.size() >= 2 && mCheckLowe )
    {
        const int j1 = queue.top();
        const double p1 = queue.top_priority();

        queue.pop();

        const int j2 = queue.top();
        const double p2 = queue.top_priority();

        if( (-p1) < mLoweRatio * (-p2) )
        {
            ret = j1;
        }
    }
    else if( queue.size() > 0 )
    {
        ret = queue.top();
    }

    if( ret >= 0 )
    {
        const double distance = cv::norm(to.keypoints[ret].pt - from.keypoints[i].pt);
        if( distance > mPredictionRadius )
        {
            ret = -1;
        }
    }

    if( ret >= 0 && check_symmetry )
    {
        const int other = matchKeyPoint(ret, from.keypoints[i].pt, to, from, false);

        if(other != i)
        {
            ret = -1;
        }
    }

    return ret;
}


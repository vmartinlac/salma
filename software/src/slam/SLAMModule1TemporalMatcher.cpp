#include <opencv2/calib3d.hpp>
#include "FinitePriorityQueue.h"
#include "SLAMModule1TemporalMatcher.h"
#include "SLAMDebug.h"

SLAMModule1TemporalMatcher::SLAMModule1TemporalMatcher(SLAMContextPtr con) :
    SLAMModule(SLAM_MODULE1_TEMPORALMATCHER, con)
{
}

bool SLAMModule1TemporalMatcher::init()
{
    SLAMConfigurationPtr conf = context()->configuration;

    mCheckSymmetry = conf->temporal_matcher.check_symmetry;
    mCheckLowe = conf->temporal_matcher.check_lowe;
    mLoweRatio = conf->temporal_matcher.lowe_ratio;
    mCheckOctave = conf->temporal_matcher.check_octave;
    mNumPreviousFrames = conf->temporal_matcher.num_previous_frames;
    //mMaxProjectedMapPointsPerView = conf->temporal_matcher.max_projected_mappoints_per_view;

    return true;
}

SLAMModuleResult SLAMModule1TemporalMatcher::operator()()
{
    std::cout << "   TEMPORAL MATCHER" << std::endl;

    SLAMReconstructionPtr rec = context()->reconstruction;

    const int N = static_cast<int>(rec->frames.size());

    if( N == 0 ) throw std::runtime_error("internal error");

    for(int i=0; i<2; i++)
    {
        std::set<int> projected_mappoints_ids;
        bool go_on = true;

        for(int j=0; go_on && N-2-j >= 0 && j < mNumPreviousFrames; j++)
        {
            processView(rec->frames[N-2-j], rec->frames[N-1], i, projected_mappoints_ids);

            go_on =
                //( projected_mappoints_ids.size() < mMaxProjectedMapPointsPerView ) &&
                ( rec->frames[N-2-j]->aligned_wrt_previous_frame );
        }

        std::cout << "      Total number of projections on VIEW_" << i << ": " << projected_mappoints_ids.size() << std::endl;
    }

    return SLAMModuleResult(false, SLAM_MODULE1_ALIGNMENT);
}

void SLAMModule1TemporalMatcher::processView(SLAMFramePtr prev_frame, SLAMFramePtr curr_frame, int view, std::set<int>& projected_mappoints_ids)
{
    SLAMView& previous_view = prev_frame->views[view];
    SLAMView& current_view = curr_frame->views[view];

    const bool dbg = context()->configuration->temporal_matcher.debug;
    std::vector<cv::DMatch> dbg_matches;

    int match_count = 0;
    int projection_count = 0;

    const int N = previous_view.keypoints.size();

    for(int i=0; i<N; i++)
    {
        const int j = matchKeyPoint(i, previous_view, current_view, mCheckSymmetry);

        if( j >= 0 )
        {
            match_count++;

            if(dbg)
            {
                dbg_matches.push_back( cv::DMatch(i, j, 1.0) );
            }
        }

        const bool take =
            ( j >= 0 ) &&
            ( bool(current_view.tracks[j].mappoint) == false ) &&
            ( bool(previous_view.tracks[i].mappoint) == true ) &&
            ( projected_mappoints_ids.count( previous_view.tracks[i].mappoint->id ) == 0 );

        if(take)
        {
            /*
            if( current_view.tracks[j].anterior_match >= 0 )
            {
                std::cerr << "Keypoint already matched!" << std::endl;
            }
            */

            //current_view.tracks[j].previous_match = i;
            //previous_view.tracks[i].next_match = j;

            projected_mappoints_ids.insert( previous_view.tracks[i].mappoint->id );

            current_view.tracks[j].mappoint = previous_view.tracks[i].mappoint;

            projection_count++;
        }
    }

    if(dbg)
    {
        cv::Mat outimg;

        cv::drawMatches(
            previous_view.image,
            previous_view.keypoints,
            current_view.image,
            current_view.keypoints,
            dbg_matches,
            outimg);

        context()->debug->saveImage(curr_frame->id, "TEMPORALMATCHING_match", outimg);
    }

    std::cout << "      Match count on FRAME_" << prev_frame->id << "/FRAME_" << curr_frame->id << "/VIEW_" << view << ": " << match_count << std::endl;
    std::cout << "      Projection count on FRAME_" << prev_frame->id << "/FRAME_" << curr_frame->id << "/VIEW_" << view << ": " << projection_count << std::endl;
}

int SLAMModule1TemporalMatcher::matchKeyPoint(int i, const SLAMView& from, const SLAMView& to, bool check_symmetry)
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

    if( ret >= 0 && check_symmetry )
    {
        const int other = matchKeyPoint(ret, to, from, false);

        if(other != i)
        {
            ret = -1;
        }
    }

    return ret;
}


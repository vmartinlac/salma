#include <opencv2/calib3d.hpp>
#include "FinitePriorityQueue.h"
#include "SLAMModuleTemporalMatcher.h"
#include "SLAMDebug.h"

SLAMModuleTemporalMatcher::SLAMModuleTemporalMatcher(SLAMContextPtr con) : SLAMModule(con)
{
}

bool SLAMModuleTemporalMatcher::init()
{
    SLAMContextPtr con = context();

    mCheckSymmetry = true; //project->getParameterBoolean("temporal_matcher_check_symmetry", true);
    mCheckLowe = true; //project->getParameterBoolean("temporal_matcher_check_lowe", true);
    mLoweRatio = true; //project->getParameterReal("temporal_matcher_lowe_ratio", 0.85);
    mCheckOctave = true; //project->getParameterBoolean("temporal_matcher_check_octave", false);

    return true;
}

void SLAMModuleTemporalMatcher::operator()()
{
    std::cout << "   TEMPORAL MATCHER" << std::endl;
    SLAMReconstructionPtr rec = context()->reconstruction;

    const int N = rec->frames.size();

    // TODO multiple frames.

    if( N >= 2 )
    {
        processView(rec->frames[N-2], rec->frames[N-1], 0);
        processView(rec->frames[N-2], rec->frames[N-1], 1);
    }
}

void SLAMModuleTemporalMatcher::processView(SLAMFramePtr prev_frame, SLAMFramePtr curr_frame, int view)
{
    SLAMView& previous_view = prev_frame->views[view];
    SLAMView& current_view = curr_frame->views[view];

    const bool dbg = context()->configuration->temporalmatcher_debug;
    std::vector<cv::DMatch> dbg_matches;

    int match_count = 0;
    int projection_count = 0;

    const int N = previous_view.keypoints.size();

    for(int i=0; i<N; i++)
    {
        const int j = matchKeyPoint(i, previous_view, current_view, mCheckSymmetry);

        if( j >= 0 )
        {
            /*
            if( current_view.tracks[j].anterior_match >= 0 )
            {
                std::cerr << "Keypoint already matched!" << std::endl;
            }
            */

            //current_view.tracks[j].previous_match = i;
            //previous_view.tracks[i].next_match = j;

            current_view.tracks[j].mappoint = previous_view.tracks[i].mappoint;

            if( current_view.tracks[j].mappoint )
            {
                projection_count++;
            }

            match_count++;

            if(dbg)
            {
                dbg_matches.push_back( cv::DMatch(i, j, 1.0) );
            }
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

    std::cout << "      Match count on view " << view << ": " << match_count << std::endl;
    std::cout << "      Projection count on view " << view << ": " << projection_count << std::endl;
}

int SLAMModuleTemporalMatcher::matchKeyPoint(int i, const SLAMView& from, const SLAMView& to, bool check_symmetry)
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


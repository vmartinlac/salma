#include <Eigen/Eigen>
#include "SLAMModuleTriangulation.h"

SLAMModuleTriangulation::SLAMModuleTriangulation(SLAMProjectPtr project) : SLAMModule(project)
{
    //mMaxNumberOfPreviousFrames = project->getParameterInteger("max_number_of_previous_frames", 5);
}

void SLAMModuleTriangulation::run(FramePtr frame)
{
    if( frame->previous_frame )
    {
        std::vector<Track>& left_tracks = frame->views[0].tracks;
        std::vector<Track>& right_tracks = frame->views[1].tracks;

        for(int i=0; i<left_tracks.size(); i++)
        {
            bool ok = true;
            
            if(ok)
            {
                ok = ( left_tracks[i].stereo_match >= 0 );
            }

            if(ok)
            {
                ok = check( frame, i, left_tracks[i].stereo_match );
            }

            if(ok)
            {
                connect(frame, i, left_tracks[i].stereo_match);
            }
        }
    }
}

void SLAMModuleTriangulation::connect(FramePtr frame, int left_kpt, int right_kpt)
{
    std::cout << left_kpt << ' ' << right_kpt << std::endl;
}

bool SLAMModuleTriangulation::check(FramePtr frame, int left_kpt0, int right_kpt0)
{
    int left_kpt = left_kpt0;
    int right_kpt = right_kpt0;

    FramePtr f = frame;
    int count = 0;

    while( count >= 0 && bool(f) && left_kpt >= 0 && right_kpt >= 0 )
    {
        if( f->views[0].tracks[left_kpt].stereo_match == right_kpt )
        {
            if( right_kpt >= 0 )
            {
                count++;
            }

            left_kpt = f->views[0].tracks[left_kpt].anterior_match;
            right_kpt = f->views[1].tracks[right_kpt].anterior_match;
            f = f->previous_frame;
        }
        else
        {
            count = -1;
        }
    }

    return (count >= 2);
}


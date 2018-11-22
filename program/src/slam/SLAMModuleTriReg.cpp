#include <Eigen/Eigen>
#include "SLAMModuleTriReg.h"

SLAMModuleTriReg::SLAMModuleTriReg(SLAMProjectPtr project) : SLAMModule(project)
{
    //const double scale_factor = project->getParameterReal("features_scale_factor", 1.1);
}

void SLAMModuleTriReg::run(FramePtr frame)
{
    if( frame->previous_frame )
    {
        /*
        const int k = 1;
        FramePtr f = frame->previous_frame;
        while( bool(f) && frame->id - f->id <= k )
        {
        }
        */

        mMapPoints.resize( frame->stereo_matches.size() );

        const int left_N = frame->previous_frame->views[0].keypoints.size();
        const int right_N = frame->previous_frame->views[1].keypoints.size();

        Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic> acc(left_N, right_N);
        acc.setZero();

        for( std::pair<int,int>& p : frame->stereo_matches )
        {
            const int i = frame->views[0].tracks[p.first].match_in_previous_frame;
            const int j = frame->views[1].tracks[p.second].match_in_previous_frame;

            if( i >= 0 && j >= 0 )
            {
                acc(i,j) |= 1;
            }
        }

        for( std::pair<int,int>& p : frame->previous_frame->stereo_matches )
        {
            const int i = p.first;
            const int j = p.second;

            acc(i,j) |= 2;
        }

        for( std::pair<int,int>& p : frame->stereo_matches )
        {
            const int i = frame->views[0].tracks[p.first].match_in_previous_frame;
            const int j = frame->views[1].tracks[p.second].match_in_previous_frame;

            if( i >= 0 && j >= 0 && acc(i,j) == 3 )
            {
                //tryTriangulate(p.first, p.second);
            }
        }
    }
}


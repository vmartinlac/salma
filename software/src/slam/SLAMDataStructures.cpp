#include "SLAMDataStructures.h"

void SLAMReconstruction::buildSegments()
{
    SLAMSegment seg;

    for(SLAMFramePtr f : frames)
    {
        if(seg.frames.empty() == false && f->aligned_wrt_previous_frame == false )
        {
            segments.push_back( std::move(seg) );
            seg = SLAMSegment();
        }

        seg.frames.push_back(f);
    }

    if(seg.frames.empty() == false)
    {
        segments.push_back( std::move(seg) );
    }
}

SLAMFrame::SLAMFrame()
{
    id = -1;
    rank_in_recording = -1;
    timestamp = 0.0;
    aligned_wrt_previous_frame = false;
    pose_covariance.setIdentity();
}

SLAMMapPoint::SLAMMapPoint()
{
    id = -1;
    position.setZero();
    position_covariance.setIdentity();
    last_seen_frame_id = -1;
}


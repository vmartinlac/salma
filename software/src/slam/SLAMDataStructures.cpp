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


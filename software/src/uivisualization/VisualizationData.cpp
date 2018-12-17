#include "VisualizationData.h"

void VisualizationData::cutListOfFramesIntoSegments()
{
    segments.clear();

    if( bool(reconstruction) == false && reconstruction->frames.empty() == false )
    {
        FrameList::iterator first = reconstruction->frames.begin();
        FrameList::iterator it = reconstruction->frames.begin();

        it++;

        while(it != reconstruction->frames.end())
        {
            FramePtr f = *it;

            if( f->aligned_wrt_previous_frame == false )
            {
                segments.push_back( std::pair<FrameList::iterator,FrameList::iterator>(first,it) );
                first = it;
            }

            it++;
        }

        segments.push_back( std::pair<FrameList::iterator,FrameList::iterator>(first,reconstruction->frames.end()) );
    }
}


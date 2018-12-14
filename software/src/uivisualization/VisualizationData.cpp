#include "VisualizationData.h"

void VisualizationData::cutListOfFramesIntoSegments()
{
    segments.clear();

    if( frames.empty() == false )
    {
        FrameList::iterator first = frames.begin();
        FrameList::iterator it = frames.begin();

        it++;

        while(it != frames.end())
        {
            FramePtr f = *it;

            if( f->aligned_wrt_previous_frame == false )
            {
                segments.push_back( std::pair<FrameList::iterator,FrameList::iterator>(first,it) );
                first = it;
            }

            it++;
        }

        segments.push_back( std::pair<FrameList::iterator,FrameList::iterator>(first,frames.end()) );
    }
}


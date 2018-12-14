#pragma once

#include "Port.h"
#include "SLAMDataStructures.h"

struct VisualizationData
{
    FrameList frames;

    std::vector< std::pair<FrameList::iterator,FrameList::iterator> > segments;

    void cutListOfFramesIntoSegments();
};

typedef Port<VisualizationData> VisualizationDataPort;


#pragma once

#include "Port.h"

struct VisualizationSettings
{
    VisualizationSettings()
    {
        show_mappoints = true;
        show_densepoints = true;
        show_rig = true;
        show_trajectory = true;
        show_projections = true;
        segment = 0;
    }

    bool show_mappoints;
    bool show_densepoints;
    bool show_rig;
    bool show_trajectory;
    bool show_projections;
    int segment;
};

typedef Port<VisualizationSettings> VisualizationSettingsPort;


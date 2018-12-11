#pragma once

#include "Port.h"

struct VisualizationSettings
{
    bool show_mappoints;
    bool show_densepoints;
    bool show_rig;
    bool show_trajectory;
    bool show_projections;
    int segment;
};

typedef Port<VisualizationSettings> VisualizationSettingsPort;


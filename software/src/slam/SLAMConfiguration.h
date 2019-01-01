#pragma once

#include <memory>

class SLAMConfiguration
{
public:

    SLAMConfiguration();
    ~SLAMConfiguration();

    // opticalflow

    int opticalflow_window_size;

    // alignment

    double alignment_ransac_inlier_rate;
    double alignment_ransac_inlier_threshold;

    // features

    double features_scale_factor;
    int features_min_width;
    int features_max_features;
    int features_patch_size;
    int features_fast_threshold;

    // stereomatcher

    bool stereomatcher_check_octave;
    bool stereomatcher_check_symmetry;
    bool stereomatcher_check_lowe;
    double stereomatcher_lowe_ratio;
    bool stereomatcher_check_epipolar;
    double stereomatcher_epipolar_threshold;

    // triangulation.

    double triangulation_min_angle_between_rays;
    bool triangulation_check_perpendicular_length;
    double triangulation_perpendicular_max_length;
    double triangulation_max_reprojection_error;
    int triangulation_track_lifetime;
    bool triangulation_use_lindstrom;
};

typedef std::shared_ptr<SLAMConfiguration> SLAMConfigurationPtr;


#pragma once

#include <memory>

class SLAMConfiguration
{
public:

    SLAMConfiguration();
    ~SLAMConfiguration();

    // features

    bool features_debug;
    double features_scale_factor;
    int features_min_width;
    int features_max_features;
    int features_patch_size;
    int features_fast_threshold;

    // temporal matcher

    bool temporalmatcher_debug;
    bool temporalmatcher_check_symmetry;
    bool temporalmatcher_check_lowe;
    double temporalmatcher_lowe_ratio;
    bool temporalmatcher_check_octave;
    int temporalmatcher_max_projected_mappoints_per_view;
    int temporalmatcher_num_previous_frames;

    // alignment

    bool alignment_debug;
    double alignment_ransac_inlier_rate;
    double alignment_ransac_inlier_threshold;

    // ekf (extended kalman filter)

    bool ekf_debug;
    double ekf_initial_position_sdev;
    double ekf_initial_attitude_sdev;
    double ekf_initial_linear_velocity_sdev;
    double ekf_initial_angular_velocity_sdev;
    int ekf_max_local_map_size;
    double ekf_prediction_linear_acceleration_sdev;
    double ekf_prediction_angular_acceleration_sdev;
    double ekf_update_projection_sdev;

    // lba (local bundle adjustment)

    /*
    bool lba_debug;
    */

    // stereomatcher

    bool stereomatcher_debug;
    bool stereomatcher_check_octave;
    bool stereomatcher_check_symmetry;
    bool stereomatcher_check_lowe;
    double stereomatcher_lowe_ratio;
    bool stereomatcher_check_epipolar;
    double stereomatcher_epipolar_threshold;

    // triangulation.

    bool triangulation_debug;
    double triangulation_min_angle_between_rays;
    bool triangulation_check_perpendicular_length;
    double triangulation_perpendicular_max_length;
    double triangulation_max_reprojection_error;
    int triangulation_track_lifetime;
    bool triangulation_use_lindstrom;
    double triangulation_min_distance_to_camera;

    // dense reconstruction.

    bool densereconstruction_debug;
};

typedef std::shared_ptr<SLAMConfiguration> SLAMConfigurationPtr;


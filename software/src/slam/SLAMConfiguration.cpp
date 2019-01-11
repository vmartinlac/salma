#include <cmath>
#include "SLAMConfiguration.h"

SLAMConfiguration::SLAMConfiguration()
{
    features_debug = false;
    features_scale_factor = 1.1;
    features_min_width = 160;
    features_max_features = 500;
    features_patch_size = 31;
    features_fast_threshold = 10;

    temporalmatcher_debug = false;
    temporalmatcher_check_symmetry = true;
    temporalmatcher_check_lowe = true;
    temporalmatcher_lowe_ratio = 0.85;
    temporalmatcher_check_octave = false;
    temporalmatcher_max_projected_mappoints_per_view = 300;
    temporalmatcher_num_previous_frames = 5;

    alignment_debug = false;
    alignment_ransac_inlier_rate = 0.8;
    alignment_ransac_inlier_threshold = 8.0;

    ekf_debug = true;
    ekf_initial_position_sdev = 0.01;
    ekf_initial_attitude_sdev = 0.01;
    ekf_initial_linear_velocity_sdev = 20.0;
    ekf_initial_angular_velocity_sdev = M_PI*0.3;
    ekf_max_local_map_size = 200;
    ekf_prediction_linear_acceleration_sdev = 10.3;
    ekf_prediction_angular_acceleration_sdev = 2.2;
    ekf_update_projection_sdev = 6.0;

    //lba_debug = false;

    stereomatcher_debug = false;
    stereomatcher_check_octave = false;
    stereomatcher_check_symmetry = true;
    stereomatcher_check_lowe = true;
    stereomatcher_lowe_ratio = 0.90;
    stereomatcher_check_epipolar = true;
    stereomatcher_epipolar_threshold = 25.0;

    triangulation_debug = false;
    triangulation_min_angle_between_rays = 1.5 * M_PI / 180.0;
    triangulation_check_perpendicular_length = false;
    triangulation_perpendicular_max_length = 0.0;
    triangulation_max_reprojection_error = 15.0;
    triangulation_use_lindstrom = true;
    triangulation_min_distance_to_camera = 1.0;

    densereconstruction_debug = false;
}

SLAMConfiguration::~SLAMConfiguration()
{
}


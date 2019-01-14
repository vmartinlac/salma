#include <cmath>
#include "SLAMConfiguration.h"

SLAMConfiguration::SLAMConfiguration()
{
    debug = true;
    pipeline = SLAM_PIPELINE_LBA;
}

SLAMConfiguration::~SLAMConfiguration()
{
}

SLAMConfigurationFeatures::SLAMConfigurationFeatures()
{
    debug = true;
    scale_factor = 1.1;
    min_width = 160;
    max_features = 500;
    patch_size = 31;
    fast_threshold = 10;
}

SLAMConfigurationTemporalMatcher::SLAMConfigurationTemporalMatcher()
{
    debug = false;
    check_symmetry = true;
    check_lowe = true;
    lowe_ratio = 0.85;
    check_octave = false;
    max_projected_mappoints_per_view = 300;
    num_previous_frames = 5;
}

SLAMConfigurationAlignment::SLAMConfigurationAlignment()
{
    debug = false;
    ransac_inlier_rate = 0.8;
    ransac_inlier_threshold = 8.0;
}

SLAMConfigurationEKF::SLAMConfigurationEKF()
{
    debug = true;
    initial_position_sdev = 0.01;
    initial_attitude_sdev = 0.01;
    initial_linear_velocity_sdev = 20.0;
    initial_angular_velocity_sdev = M_PI*0.3;
    max_local_map_size = 200;
    prediction_linear_acceleration_sdev = 10.3;
    prediction_angular_acceleration_sdev = 2.2;
    update_projection_sdev = 6.0;
}

SLAMConfigurationLBA::SLAMConfigurationLBA()
{
    debug = false;
    num_previous_frames = 12;
    sigma_projection = 2.0;
}

SLAMConfigurationStereoMatcher::SLAMConfigurationStereoMatcher()
{
    debug = false;
    check_octave = false;
    check_symmetry = true;
    check_lowe = true;
    lowe_ratio = 0.90;
    check_epipolar = true;
    epipolar_threshold = 25.0;
}

SLAMConfigurationTriangulation::SLAMConfigurationTriangulation()
{
    debug = false;
    min_angle_between_rays = 1.5 * M_PI / 180.0;
    check_perpendicular_length = false;
    perpendicular_max_length = 0.0;
    max_reprojection_error = 15.0;
    use_lindstrom = true;
    min_distance_to_camera = 1.0;
    sigma_proj_left = 0.5;
    sigma_proj_right = 0.5;
}

SLAMConfigurationDenseReconstruction::SLAMConfigurationDenseReconstruction()
{
    debug = false;
}


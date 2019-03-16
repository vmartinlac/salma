#include <cmath>
#include "SLAMConfiguration.h"

SLAMConfiguration::SLAMConfiguration()
{
    pipeline = SLAM_PIPELINE1;
}

SLAMConfiguration::~SLAMConfiguration()
{
}

SLAMConfigurationRectification::SLAMConfigurationRectification()
{
    debug = false;
}

SLAMConfigurationFeatures::SLAMConfigurationFeatures()
{
    debug = false;
    first_level = 0;
    num_levels = 5;
    scale_factor = 1.2;
    max_features = 300;
    patch_size = 31;
    fast_threshold = 10;
}

SLAMConfigurationTemporalMatcher::SLAMConfigurationTemporalMatcher()
{
    debug = false;
    check_symmetry = false;
    check_lowe = true;
    lowe_ratio = 0.75;
    check_octave = false;
    //max_projected_mappoints_per_view = 300;
    max_descriptor_distance = 400.0;
    num_previous_frames = 2;
}

SLAMConfigurationAlignment::SLAMConfigurationAlignment()
{
    debug = false;
    ransac_inlier_rate = 0.75;
    ransac_inlier_threshold = 12.0;
}

SLAMConfigurationKFS::SLAMConfigurationKFS()
{
    debug = false;
    translation_threshold = 2.0;
    angle_threshold = 1.0*M_PI;
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
    max_previous_frames = 5;
    max_mappoints = 400;
    sigma_projection = 1.5;
    verbose = false;
    max_steps = 20;
}

SLAMConfigurationStereoMatcher::SLAMConfigurationStereoMatcher()
{
    debug = true;
    check_octave = false;
    check_symmetry = true;
    check_lowe = true;
    lowe_ratio = 0.80;
    check_epipolar = true;
    epipolar_threshold = 7.0;
    max_descriptor_distance = 300.0;
}

SLAMConfigurationTriangulation::SLAMConfigurationTriangulation()
{
    debug = false;
    min_angle_between_rays = 1.5 * M_PI / 180.0;
    check_perpendicular_length = false;
    perpendicular_max_length = 0.0;
    max_reprojection_error = 4.0;
    use_lindstrom = true;
    min_distance_to_camera = 1.0;
    sigma_proj_left = 0.5;
    sigma_proj_right = 0.5;
}

SLAMConfigurationDenseReconstruction::SLAMConfigurationDenseReconstruction()
{
    debug = true;
    enabled = true;
}


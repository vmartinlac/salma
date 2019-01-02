#include <cmath>
#include "SLAMConfiguration.h"

SLAMConfiguration::SLAMConfiguration()
{
    debug = true;

    opticalflow_window_size = 21;

    alignment_ransac_inlier_rate = 0.8;
    alignment_ransac_inlier_threshold = 8.0;

    features_scale_factor = 1.1;
    features_min_width = 160;
    features_max_features = 500;
    features_patch_size = 31;
    features_fast_threshold = 10;

    stereomatcher_check_octave = false;
    stereomatcher_check_symmetry = true;
    stereomatcher_check_lowe = true;
    stereomatcher_lowe_ratio = 0.85;
    stereomatcher_check_epipolar = false;
    stereomatcher_epipolar_threshold = 10.0;

    triangulation_min_angle_between_rays = 3.0 * M_PI / 180.0;
    triangulation_check_perpendicular_length = false;
    triangulation_perpendicular_max_length = 0.0;
    triangulation_max_reprojection_error = 2.0;
    triangulation_track_lifetime = 4;
    triangulation_use_lindstrom = true;
}

SLAMConfiguration::~SLAMConfiguration()
{
}


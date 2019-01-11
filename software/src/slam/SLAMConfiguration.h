#pragma once

#include <memory>

struct SLAMConfigurationFeatures
{
    SLAMConfigurationFeatures();

    bool debug;
    double scale_factor;
    int min_width;
    int max_features;
    int patch_size;
    int fast_threshold;
};

struct SLAMConfigurationTemporalMatcher
{
    SLAMConfigurationTemporalMatcher();

    bool debug;
    bool check_symmetry;
    bool check_lowe;
    double lowe_ratio;
    bool check_octave;
    int max_projected_mappoints_per_view;
    int num_previous_frames;
};

struct SLAMConfigurationAlignment
{
    SLAMConfigurationAlignment();

    bool debug;
    double ransac_inlier_rate;
    double ransac_inlier_threshold;
};

struct SLAMConfigurationEKF
{
    SLAMConfigurationEKF();

    bool debug;
    double initial_position_sdev;
    double initial_attitude_sdev;
    double initial_linear_velocity_sdev;
    double initial_angular_velocity_sdev;
    int max_local_map_size;
    double prediction_linear_acceleration_sdev;
    double prediction_angular_acceleration_sdev;
    double update_projection_sdev;
};

/*
struct SLAMConfigurationLBA
{
    SLAMConfigurationLBA();

    bool debug;
};
*/

struct SLAMConfigurationStereoMatcher
{
    SLAMConfigurationStereoMatcher();

    bool debug;
    bool check_octave;
    bool check_symmetry;
    bool check_lowe;
    double lowe_ratio;
    bool check_epipolar;
    double epipolar_threshold;
};

struct SLAMConfigurationTriangulation
{
    SLAMConfigurationTriangulation();

    bool debug;
    double min_angle_between_rays;
    bool check_perpendicular_length;
    double perpendicular_max_length;
    double max_reprojection_error;
    int track_lifetime;
    bool use_lindstrom;
    double min_distance_to_camera;
};

/*
struct SLAMConfigurationDenseReconstruction
{
    SLAMConfigurationDenseReconstruction();

    bool debug;
};
*/

class SLAMConfiguration
{
public:

    SLAMConfiguration();
    ~SLAMConfiguration();

    SLAMConfigurationFeatures features;
    SLAMConfigurationTemporalMatcher temporal_matcher;
    SLAMConfigurationAlignment alignment;
    SLAMConfigurationEKF ekf;
    //SLAMConfigurationLBA lba;
    SLAMConfigurationStereoMatcher stereo_matcher;
    SLAMConfigurationTriangulation triangulation;
    //SLAMConfigurationDenseReconstruction dense_reconstruction;
};

typedef std::shared_ptr<SLAMConfiguration> SLAMConfigurationPtr;


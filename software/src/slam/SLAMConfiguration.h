#pragma once

#include <memory>

enum SLAMPipeline
{
    SLAM_PIPELINE1,
    SLAM_PIPELINE2,
};

struct SLAMConfigurationRectification
{
    SLAMConfigurationRectification();

    bool debug;
};

struct SLAMConfigurationFeatures
{
    SLAMConfigurationFeatures();

    bool debug;
    int first_level;
    int num_levels;
    double scale_factor;
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
    double max_descriptor_distance;
};

struct SLAMConfigurationAlignment
{
    SLAMConfigurationAlignment();

    bool debug;
    double ransac_inlier_rate;
    double ransac_inlier_threshold;
};

struct SLAMConfigurationKFS
{
    SLAMConfigurationKFS();

    bool debug;
    double translation_threshold;
    double angle_threshold;
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

struct SLAMConfigurationLBA
{
    SLAMConfigurationLBA();

    bool debug;
    bool verbose;
    int max_steps;
    int max_previous_frames;
    int max_mappoints;
    double sigma_projection;
};

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
    double max_descriptor_distance;
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
    double sigma_proj_left;
    double sigma_proj_right;
};

struct SLAMConfigurationDenseReconstruction
{
    SLAMConfigurationDenseReconstruction();

    bool debug;
    bool enabled;
};

class SLAMConfiguration
{
public:

    SLAMConfiguration();
    ~SLAMConfiguration();

    SLAMPipeline pipeline;

    SLAMConfigurationRectification rectification;
    SLAMConfigurationFeatures features;
    SLAMConfigurationTemporalMatcher temporal_matcher;
    SLAMConfigurationAlignment alignment;
    SLAMConfigurationEKF ekf;
    SLAMConfigurationLBA lba;
    SLAMConfigurationKFS kfs;
    SLAMConfigurationStereoMatcher stereo_matcher;
    SLAMConfigurationTriangulation triangulation;
    SLAMConfigurationDenseReconstruction dense_reconstruction;
};

typedef std::shared_ptr<SLAMConfiguration> SLAMConfigurationPtr;


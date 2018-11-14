#pragma once

class SLAMParameters
{
public:

    // features.
    int num_octaves;
    double scale_factor;

    // stereo matching.
    double epipolar_threshold;
    double lowe_ratio;
};


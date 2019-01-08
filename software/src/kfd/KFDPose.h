#pragma once

#include <sophus/se3.hpp>
#include "Port.h"

struct KFDPoseData
{
    Sophus::SE3d camera_to_world;
};

typedef Port<KFDPoseData> KFDPosePort;


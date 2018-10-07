#pragma once

#include <string>
#include "Port.h"
#include "Camera.h"

struct CameraCalibrationParametersData
{
    std::string output_path;
    CameraPtr camera;
};

typedef Port<CameraCalibrationParametersData> CameraCalibrationParameters;


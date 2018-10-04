#pragma once

#include "Port.h"
#include <QDir>
#include "Camera.h"

struct ParametersData
{
    QDir output_directory;
    CameraPtr camera;
};

typedef Port<ParametersData> Parameters;


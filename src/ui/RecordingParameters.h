#pragma once

#include "Port.h"
#include <QDir>
#include "Camera.h"

struct RecordingParametersData
{
    QDir output_directory;
    CameraPtr camera;
};

typedef Port<RecordingParametersData> RecordingParameters;


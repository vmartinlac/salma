#pragma once

#include <QObject>
#include <opencv2/core.hpp>
#include "Port.h"

struct OutputData
{
    cv::Mat image;
    QString camera_name;
    QString output_directory;
    int frame_count;
};

typedef Port<OutputData> Output;


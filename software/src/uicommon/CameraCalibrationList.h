#pragma once

#include <QString>
#include <vector>

struct CameraCalibrationListItem
{
    int id;
    QString name;
    QString date;
};

typedef std::vector<CameraCalibrationListItem> CameraCalibrationList;


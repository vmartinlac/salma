#pragma once

#include <QString>
#include <vector>

struct RigCalibrationListItem
{
    int id;
    QString name;
    QString date;
};

typedef std::vector<RigCalibrationListItem> RigCalibrationList;


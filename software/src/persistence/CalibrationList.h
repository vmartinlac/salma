#pragma once

#include <QString>
#include <vector>

struct CalibrationListItem
{
    int id;
    QString name;
    QString date;
};

typedef std::vector<CalibrationListItem> CalibrationList;


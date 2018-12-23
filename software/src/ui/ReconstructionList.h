#pragma once

#include <QString>
#include <vector>

struct ReconstructionListItem
{
    int id;
    QString name;
    QString date;
};

typedef std::vector<ReconstructionListItem> ReconstructionList;


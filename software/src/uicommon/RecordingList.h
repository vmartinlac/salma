#pragma once

#include <QString>
#include <vector>

struct RecordingListItem
{
    int id;
    QString name;
    QString date;
};

typedef std::vector<RecordingListItem> RecordingList;


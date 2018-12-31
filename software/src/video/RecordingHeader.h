#pragma once

#include <vector>
#include <string>
#include <QDir>

struct RecordingHeaderView
{
    QString filename;
};

struct RecordingHeaderFrame
{
    double timestamp;
};

class RecordingHeader
{
public:

    RecordingHeader()
    {
        id = -1;
        num_views = 0;
        num_frames = 0;
    }

    int id;
    std::string name;
    std::string date;

    int num_views;
    int num_frames;
    QDir directory;
    std::vector<RecordingHeaderFrame> frames;
    std::vector<RecordingHeaderView> views;
};

typedef std::shared_ptr<RecordingHeader> RecordingHeaderPtr;


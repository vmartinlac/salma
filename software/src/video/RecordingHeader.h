#pragma once

#include <vector>

struct RecordingHeaderView
{
    QString filename;
};

struct RecordingHeaderFrame
{
    double timestamp;
};

struct RecordingHeader
{
    int num_views;
    int num_frames;
    QDir directory;
    std::vector<RecordingHeaderFrame> frames;
    std::vector<RecordingHeaderView> views;
};

typedef std::shared_ptr<RecordingHeader> RecordingHeaderPtr;


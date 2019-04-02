#pragma once

#include <vector>
#include <string>
#include <memory>
#include <opencv2/core.hpp>
#include <QDir>

class RecordingHeader
{
public:

    RecordingHeader()
    {
        id = -1;
    }

    int id;
    std::string name;
    std::string date;

    std::vector<double> timestamps;
    std::vector<cv::Size> views;
    QDir directory;

public:

    QString getImageFileName(int frame, int view);

    int num_views()
    {
        return static_cast<int>(views.size());
    }

    int num_frames()
    {
        return static_cast<int>(timestamps.size());
    }
};

typedef std::shared_ptr<RecordingHeader> RecordingHeaderPtr;


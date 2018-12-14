
#pragma once

#include <QDir>
#include <future>
#include <memory>
#include <fstream>
#include <iostream>
#include <string>
#include "VideoSource.h"

class VideoReader : public VideoSource
{
public:

    VideoReader(int views);
    ~VideoReader();

    void setPath(const std::string& path);

    std::string getHumanName() override;

    bool open() override;
    void close() override;

    void trigger() override;
    void read(Image& image) override;

    int getNumberOfCameras() override;

protected:

    int mNumViews;
    QDir mDirectory;
    std::ifstream mCSVFile;
    bool mTriggered;
    std::future<Image> mNextImage;
};

typedef std::shared_ptr<VideoReader> VideoReaderPtr;


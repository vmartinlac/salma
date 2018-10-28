
#pragma once

#include <memory>
#include <fstream>
#include <iostream>
#include <string>
#include "VideoSource.h"

class VideoReader : public VideoSource
{
public:

    VideoReader();
    ~VideoReader();

    void setFileName(const std::string& filename);

    std::string getHumanName() override;

    bool open() override;
    void close() override;

    void trigger() override;
    void read(Image& image) override;

    int getNumberOfCameras() override;

protected:

    std::string mFileName;
    std::ifstream mCSVFile;
    int mNumViews;
};

typedef std::shared_ptr<VideoReader> VideoReaderPtr;


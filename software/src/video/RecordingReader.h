
#pragma once

#include <opencv2/videoio.hpp>
#include <future>
#include "VideoSource.h"
#include "RecordingHeader.h"

class RecordingReader : public VideoSource
{
public:

    RecordingReader(RecordingHeaderPtr header, bool unused);
    ~RecordingReader() override;

    std::string getHumanName() override;

    bool open() override;
    void close() override;

    void trigger() override;
    void read(Image& image) override;

    int getNumberOfCameras() override;

    void seek(int frame);

    RecordingHeaderPtr getHeader();

protected:

    bool mTriggered;
    RecordingHeaderPtr mHeader;
    cv::VideoCapture mVideo;
};

typedef std::shared_ptr<RecordingReader> RecordingReaderPtr;


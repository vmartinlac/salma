
#pragma once

#include <future>
#include "VideoSource.h"
#include "RecordingHeader.h"

class RecordingReader : public VideoSource
{
public:

    RecordingReader(RecordingHeaderPtr header, bool asynchronous);
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

    static Image loadImage(RecordingHeaderPtr rec, int rank);

protected:

    bool mAsynchronousLoading;
    RecordingHeaderPtr mHeader;
    bool mIsOpen;
    int mNextFrame;
    std::future<Image> mNextImage;
};

typedef std::shared_ptr<RecordingReader> RecordingReaderPtr;



#pragma once

#include "RecordingHeader.h"
#include "Image.h"

class RecordingWriter
{
public:

    RecordingWriter(const QDir& dir);

    ~RecordingWriter();

    bool write(const Image& image);

    RecordingHeaderPtr getHeader();

    void erase();

protected:

    RecordingHeaderPtr mHeader;
};

typedef std::shared_ptr<RecordingWriter> RecordingWriterPtr;


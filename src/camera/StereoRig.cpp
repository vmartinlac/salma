#include "StereoRig.h"

class StereoRecording : public StereoRig
{
public:

    StereoRecording(const std::string& filename);

    bool open() override;
    void close() override;

    bool trigger() override;
    bool read(StereoImage& to) override;

protected:

    std::string mFilename;
};

bool StereoRecording::open()
{
    return false;
}

void StereoRecording::close()
{
}

bool StereoRecording::trigger()
{
    return false;
}

bool StereoRecording::read(StereoImage& to)
{
    return false;
}

StereoRigPtr StereoRig::createFromRecording(const std::string& filename)
{
    return StereoRigPtr(new StereoRecording(filename));
}


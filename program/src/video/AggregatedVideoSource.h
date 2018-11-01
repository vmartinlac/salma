#pragma once

#include <vector>
#include "VideoSource.h"

class AssembledVideoSource : public VideoSource
{
public:

    AssembledVideoSource();
    ~AssembledVideoSource() override;

    std::string getHumanName() override;

    bool open() override;
    void close() override;

    void trigger() override;
    void read(Image& image) override;

    int getNumberOfCameras() override;

    void setVideoSources(const std::vector<VideoSourcePtr>& video_sources);

protected:

    std::vector<VideoSourcePtr> mVideoSources;
};


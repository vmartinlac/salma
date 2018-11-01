#pragma once

#include <string>
#include "VideoSource.h"

class VideoSystem
{
public:

    static VideoSystem* instance();

    virtual bool initialize() = 0;
    virtual void finalize() = 0;

public:

    virtual ~VideoSystem();

    virtual int getNumberOfGenICamCameras();
    virtual std::string getNameOfGenICamCamera(int idx);

    virtual VideoSourcePtr createVideoSourceGenICamMono(int camera_idx);
    virtual VideoSourcePtr createVideoSourceGenICamStereo(int left_camera_idx, int right_camera_id);

    virtual VideoSourcePtr createVideoSourceOpenCV(int id);
    virtual VideoSourcePtr createVideoSourceOpenCV(const std::string& filename);

    virtual VideoSourcePtr createVideoSourceFromFileMono(const std::string& path);
    virtual VideoSourcePtr createVideoSourceFromFileStereo(const std::string& path);

    virtual VideoSourcePtr createVideoSourceMockMono();
    virtual VideoSourcePtr createVideoSourceMockStereo();

protected:

    VideoSystem();

private:

    static std::unique_ptr<VideoSystem> mInstance;
};


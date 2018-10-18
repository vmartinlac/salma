#pragma once

#include <vector>
#include "Camera.h"
#include "Trigger.h"
#include "VideoSource.h"

class GeneralVideoSource : public VideoSource
{
public:

    GeneralVideoSource();
    ~GeneralVideoSource() override;

    void set(CameraPtr camera, TriggerPtr trigger);
    void set(CameraPtr camera0, CameraPtr camera1, TriggerPtr trigger);
    void set(CameraPtr camera0, CameraPtr camera1, CameraPtr camera2, TriggerPtr trigger);

    std::string getHumanName() override;

    bool open() override;
    void close() override;

    void trigger() override;
    void read(Image& image) override;

    int getNumberOfCameras() override;

protected:

    std::vector<CameraPtr> mCameras;
    TriggerPtr mTrigger;
};

typedef std::shared_ptr<GeneralVideoSource> GeneralVideoSourcePtr;


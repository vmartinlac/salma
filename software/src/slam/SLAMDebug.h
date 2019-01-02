#pragma once

#include <memory>
#include <string>
#include <opencv2/core.hpp>
#include "SLAMConfiguration.h"

class SLAMDebug
{
public:

    SLAMDebug( SLAMConfigurationPtr conf );
    ~SLAMDebug();

    bool init();
    void saveImage(int frame, const std::string& name, const cv::Mat& image);

protected:

    int mImageCount;
    SLAMConfigurationPtr mConfiguration;
};

typedef std::shared_ptr<SLAMDebug> SLAMDebugPtr;


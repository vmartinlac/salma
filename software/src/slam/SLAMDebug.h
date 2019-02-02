#pragma once

#include <memory>
#include <map>
#include <string>
#include <opencv2/core.hpp>
#include "SLAMConfiguration.h"
#include <QDir>

class SLAMDebug
{
public:

    SLAMDebug( SLAMConfigurationPtr conf );
    ~SLAMDebug();

    bool init();
    void saveImage(int frame, const std::string& name, const cv::Mat& image);

    std::string describeOpenCVMat(const cv::Mat& mat);

    /*
    void beginReconstruction();
    void endReconstruction();
    void beginFrame(int id);
    void endFrame();
    void beginModule(const std::string& name);
    void endModule();
    void saveImage(const std::string& name, const cv::Mat& image);
    void writeText(const str::string& text);
    */

protected:

    int mImageCount;
    SLAMConfigurationPtr mConfiguration;
    QDir mDir;
    std::map<int,std::string> mOpenCVTypes;
};

typedef std::shared_ptr<SLAMDebug> SLAMDebugPtr;


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

    std::string getNextSaveFileName(int frame, const std::string& basename);

    void saveImage(int frame, const std::string& name, const cv::Mat& image);
    void savePointCloud(int frame, const std::string& name, const std::vector<cv::Point3f>& cloud);

    std::string describeOpenCVMat(const cv::Mat& mat);

protected:

    int mFileCount;
    SLAMConfigurationPtr mConfiguration;
    QDir mDir;
    std::map<int,std::string> mOpenCVTypes;
};

typedef std::shared_ptr<SLAMDebug> SLAMDebugPtr;


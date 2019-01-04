#include <opencv2/imgcodecs.hpp>
#include <iomanip>
#include <sstream>
#include "SLAMDebug.h"

SLAMDebug::SLAMDebug( SLAMConfigurationPtr config )
{
    mImageCount = 0;
    mConfiguration = std::move(config);
}

SLAMDebug::~SLAMDebug()
{
}

bool SLAMDebug::init()
{
    mImageCount = 0;
    return true;
}

void SLAMDebug::saveImage(int frame, const std::string& name, const cv::Mat& image)
{
    std::stringstream s;
    s << std::setfill('0') << std::setw(6) << mImageCount << '_' << frame << '_' << name << ".png";

    cv::imwrite(s.str(), image);

    mImageCount++;
}


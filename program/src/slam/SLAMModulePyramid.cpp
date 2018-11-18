#include "SLAMModulePyramid.h"

SLAMModulePyramid::SLAMModulePyramid(SLAMProjectPtr project) : SLAMModule(project)
{
}

void SLAMModulePyramid::process(const cv::Mat& input, SLAMPyramid& pyramid)
{
    throw std::runtime_error("not implemented!"); // TODO
}


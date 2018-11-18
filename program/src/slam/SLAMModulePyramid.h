
#pragma once

#include "SLAMPyramid.h"
#include "SLAMModule.h"

class SLAMModulePyramid : public SLAMModule
{
public:

    SLAMModulePyramid(SLAMProjectPtr project);

    void process(const cv::Mat& input, SLAMPyramid& pyramid);
};

typedef std::shared_ptr<SLAMModulePyramid> SLAMModulePyramidPtr;


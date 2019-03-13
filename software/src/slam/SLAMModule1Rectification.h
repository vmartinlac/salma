
#pragma once

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <utility>
#include <cmath>
#include "SLAMDataStructures.h"
#include "SLAMModule.h"

class SLAMModule1Rectification : public SLAMModule
{
public:

    SLAMModule1Rectification(SLAMContextPtr con);
    ~SLAMModule1Rectification() override;

    bool init() override;
    SLAMModuleResult operator()() override;
};


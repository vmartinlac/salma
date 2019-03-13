
#pragma once

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <utility>
#include <cmath>
#include "SLAMDataStructures.h"
#include "SLAMModule.h"

class SLAMModule1KFS : public SLAMModule
{
public:

    SLAMModule1KFS(SLAMContextPtr con);
    ~SLAMModule1KFS() override;

    bool init() override;
    SLAMModuleResult operator()() override;
};


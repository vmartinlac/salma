
#pragma once

#include <vector>
#include <opencv2/core.hpp>
#include <QDir>
#include <QTime>
#include "Tracker.h"
#include "VideoSource.h"
#include "Operation.h"

class ReconstructionOperation : public Operation
{
public:

    ReconstructionOperation();

    ~ReconstructionOperation() override;

    const char* getName() override;

    bool before() override;
    bool step() override;
    void after() override;

    bool success() override;
    bool saveResult(Project* project) override;
    void discardResult() override;

public:

    std::string mReconstructionName;
};


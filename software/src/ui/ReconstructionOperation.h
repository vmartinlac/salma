
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

    bool uibefore(QWidget* parent, Project* project) override;
    bool before() override;
    bool step() override;
    void after() override;
    void uiafter(QWidget* parent, Project* project) override;

public:

    std::string mReconstructionName;
};


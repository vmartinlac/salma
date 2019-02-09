
#pragma once

#include <vector>
#include <opencv2/core.hpp>
#include <QDir>
#include <QTime>
#include "VideoSource.h"
#include "Operation.h"
#include "SLAMEngine.h"
#include "SLAMConfiguration.h"
#include "RecordingReader.h"

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
    RecordingHeaderPtr mRecordingHeader;
    StereoRigCalibrationDataPtr mCalibration;
    SLAMConfigurationPtr mConfiguration;

protected:

    RecordingReaderPtr mRecordingReader;
    SLAMReconstructionPtr mReconstruction;
    SLAMEnginePtr mEngine;
    int mNextFrame;
};

